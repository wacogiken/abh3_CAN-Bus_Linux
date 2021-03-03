/******************************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Waco Giken Co., Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Waco Giken nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*!****************************************************************************
@file           pack_float.c
*******************************************************************************
@brief          PACK_FLOAT ライブラリ（ソース）
*******************************************************************************
@date           2009.9.8
@author         m.hasegawa
@note           ・初版
*******************************************************************************
@date           2009.9.9
@author         m.hasegawa
@note           ・実数から PACK_FLOAT 変換時小数点を工夫
*******************************************************************************
@date           2009.9.10
@author         m.hasegawa
@note           ・実数から PACK_FLOAT 変換で0.0のとき永久ループになるのを修正
*******************************************************************************
@date           2010.2.23
@author         y.masubuchi
@note           ・有効桁6桁以内の数値(0.00001～999999)はそのまま表示。
@note           ・小数点以下は下位側ゼロサプレス。(1.10000 -> 1.1)
@note           ・指数がある場合は整数部も下位側ゼロサプレス。 (110000e2 -> 11e6 or 1.1e7) 
*******************************************************************************
@date           2010.3.25
@author         m.hasegawa
@note           ・flt_to_pkflt() で-9.155273e-4を変換したとき shousuutenn がオーバフローするのを修正。
*******************************************************************************
@date           2010.4.7
@author         m.hasegawa
@note           ・flt_to_pkflt で四捨五入追加。これはコンパイラが実数→整数変換時ゼロ方向へ丸め時にのみ使用する。
*******************************************************************************
@date           2015.3.9
@author         y.masubuchi
@note           ・str_to_pkflt 指数の範囲を-38～38の範囲で制限するよう修正。
@note           ・pkflt_to_flt 指数の範囲を-38～38の範囲で制限するよう修正。
*******************************************************************************
@date           2016.01.20
@author         H.SHINGA
@note           ・HEW（Renesas）および Visual C++（Microsoft）コンパイラへの対応。
@note           ・無限大および非数に対応する様に変更。（_math.h が必要）
@note           ・指数の範囲制限を削除し、大きい値は無限大となる様に変更。
@note           ・関数の複雑度に対する改善。（関数の分散化）
@note           ・コメント文のDoxygen対応
*******************************************************************************
@date           2016.01.21
@author         H.SHINGA
@note           ・コメント文追記
******************************************************************************/

/******************************************************************************
  インクルードファイル
******************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "_math.h"

#include "pack_float.h"

/******************************************************************************
  構成定義
******************************************************************************/

//  実数 → 整数変換時ゼロ方向へ丸め

//  HEW
#ifdef    __HITACHI__

//  HEW によって定義されるマクロを利用する
#ifdef    _RON
//  CPU 丸め方式 - Nearest
#else   //_RON
//  CPU 丸め方式 - Zero
#define  CPU_ROUND_ZERO  //!<丸め方式：ゼロ方向
#endif  //_RON

#endif  //__HITACHI__

//  Visual C++
#ifdef    _MSC_VER

#define  CPU_ROUND_ZERO  //!<丸め方式：ゼロ方向

#endif  //_MSC_VER

/******************************************************************************
  定数定義
******************************************************************************/

//  PACK_FLOAT型の定数

//  ゼロ定数の定義

PACK_FLOAT_DUAL const  _pkflt_zero = { 0 };  //!<PACK_FLOAT型、ゼロ定数

//  無限大定数の定義

PACK_FLOAT_DUAL const  _pkflt_pinf = { PACK_FLOAT_KASUU_PINF };  //!<PACK_FLOAT型、正の無限大定数

PACK_FLOAT_DUAL const  _pkflt_ninf = { PACK_FLOAT_KASUU_NINF };  //!<PACK_FLOAT型、負の無限大定数

//  非数定数の定義

PACK_FLOAT_DUAL const  _pkflt_nan  = { 0xFFF00001 };  //!<PACK_FLOAT型、非数定数

/******************************************************************************
  ローカル関数宣言
******************************************************************************/

/*  10進数文字列を整数に変換する関数（有効数字上位6桁、値のみ）
@param[in,out]  p_str       [in]10進数文字列の変換開始位置(char *)が格納されているポインタ(char **)、[out]関数後は変換終了位置が格納される
@param[in,out]  p_x         10進数文字列の変換結果値を格納するポインタ、関数後は変換結果値が元あった値に追加格納される
@param[out]     p_valid     10進数文字列の変換有効桁数が格納されるポインタ
@return                     数値の桁数（ただし先頭の0は省く）
@attention                  文字列中の符号や小数点記号を検出した段階でこの関数は終了する
*/
static int fl_digit( const char **p_str, long *p_x, int *p_valid );

/*  数値文字列を仮数値と指数値に変換する関数（有効数字上位6桁、符号対応、小数点位置対応）
@param[in,out]  p_str   [in]10進数文字列の変換開始位置(char *)が格納されているポインタ(char **)、[out]関数後は変換終了位置が格納される
@param[out]     p_x     10進数文字列の変換結果値を格納するポインタ、関数後は変換結果の有効数字上位6桁の仮数値が格納される
@param[in,out]  p_dp    [in]10進数文字列の変換結果の指数値を格納するポインタ、ポインタがNULLであった場合は小数点('.')を検出した場合は異常を返し、[out]NULLでなかった場合は関数後は変換結果の指数値が格納される
@return                 変換終了位置のキャラクタ
@note                   p_dpにNULLを渡す事で、小数点非対応の関数として動作させられる。
*/
static char fl( const char **p_str, long *p_x, int *p_dp );

/*  数値を数値文字列へ変換する変換関数（左詰め、正の値のみ、変動桁数）
@param[out]    temp_top 数値文字列への変換結果を格納するテンポラリ領域への先頭ポインタ
@param[in,out] p_point  テンポラリ領域の現在位置が格納されているポインタ
@param[in]     x        変換元となる数値
@return                 変換されきれなかった残りの数値
*/
static long cv_x( char *temp_top, int *p_point, long x );

/*  数値を数値文字列へ変換する変換関数（左詰め、正の値のみ、固定桁数）
@param[out]    temp_top 数値文字列への変換結果を格納するテンポラリ領域への先頭ポインタ
@param[in,out] p_point  テンポラリ領域の現在位置が格納されているポインタ
@param[in]     x        変換元となる数値
@param[in]     digit    変換する桁数
@return                 変換されきれなかった残りの数値
*/
static long cv_digit( char *temp_top, int *p_point, long x, int digit );

/*  数値を数値文字列へ変換する変換関数（符号対応、小数点位置対応）
@param[out]    str      数値文字列への変換結果を格納するポインタ
@param[in]     x        変換元となる数値
@param[in]     dp       変換する数値の小数点位置
@param[in]     sign     正の値の場合の符号強制
@return                 変換された文字列の長さ
*/
static int cv( char *str, long x, long dp, int sign );

/*  PACK_FLOAT型 → 数値文字列 変換関数（有限値限定）
@param[in]      pkflt   変換元となるPACK_FLOAT型浮動小数点値
@param[out]     str     数値文字列への変換結果を格納するポインタ
@return                 渡された str と同じ場所を指すポインタ
@warning                変換元となるPACK_FLOAT型浮動小数点値が有限値でない場合、期待した結果は得られない
*/
static char *pkflt_to_str_finite( PACK_FLOAT pkflt, char *str );

/*! 10のべき乗値乗算関数
@param[in]  flt     実数
@param[in]  _e      指数
@return             実数に底を10とした指数のべき乗を掛けた結果
*/
static float pkflt_to_flt_mul_exp( float flt, long _e );

/*! 10のべき乗値除算関数
@param[in]  flt     実数
@param[in]  _e      指数
@return             実数に底を10とした指数のべき乗で割った結果
*/
static float pkflt_to_flt_div_exp( float flt, long _e );

/*  PACK_FLOAT型 → float型 変換関数 （有限値限定）
@param[in]  pkflt   PACK_FLOAT型の値
@return             float型に変換された値
*/
static float pkflt_to_flt_finite( PACK_FLOAT pkflt );

/*  float型 → PACK_FLOAT型 変換関数 （有限値限定）
@param[in]  flt     float型の値
@return             PACK_FLOAT型に変換された値
*/
static PACK_FLOAT flt_to_pkflt_finite( float flt );

/******************************************************************************
  ローカル関数
******************************************************************************/

/*! 10進数文字列を整数に変換する関数（有効数字上位6桁、値のみ）
@param[in,out]  p_str       [in]10進数文字列の変換開始位置(char *)が格納されているポインタ(char **)、[out]関数後は変換終了位置が格納される
@param[in,out]  p_x         10進数文字列の変換結果値を格納するポインタ、関数後は変換結果値が元あった値に追加格納される
@param[out]     p_valid     10進数文字列の変換有効桁数が格納されるポインタ
@return                     数値の桁数（ただし先頭の0は省く）
@attention                  文字列中の符号や小数点記号を検出した段階でこの関数は終了する
*/
static int fl_digit( const char **p_str, long *p_x, int *p_valid )
{
	//  変換結果値
	long _x;
	//  変換桁数
	int digit;
	//  変換有効桁数
	int _valid;
	//  現在のキャラクタによる数値
	int _n;
	//  現在チェック中のキャラクタ
	char _c;

	_x = *p_x;
	_valid = *p_valid;
	digit = 0;
	//  変換処理
	while ( isdigit( _c = **p_str ) ) {
		_n = _c - '0';
		if ( _x <= 99999 ) {
			//  6桁目以内
			_x = _x * 10 + _n;
			//  有効桁数の加算
			if ( _x || _valid ) {
				_valid++;
			}
		}
		//  桁数の加算
		if ( _x ) {
			digit++;
		}
		( *p_str )++;
	}
	*p_x = _x;
	*p_valid = _valid;
	return ( digit );
}

/*! 数値文字列を仮数値と指数値に変換する関数（有効数字上位6桁、符号対応、小数点位置対応）
@param[in,out]  p_str   [in]10進数文字列の変換開始位置(char *)が格納されているポインタ(char **)、[out]関数後は変換終了位置が格納される
@param[out]     p_x     10進数文字列の変換結果値を格納するポインタ、関数後は変換結果の有効数字上位6桁の仮数値が格納される
@param[in,out]  p_dp    [in]10進数文字列の変換結果の指数値を格納するポインタ、ポインタがNULLであった場合は小数点('.')を検出した場合は異常を返し、[out]NULLでなかった場合は関数後は変換結果の指数値が格納される
@return                 変換終了位置のキャラクタ
@note                   p_dpにNULLを渡す事で、小数点非対応の関数として動作させられる。
*/
static char fl( const char **p_str, long *p_x, int *p_dp )
{
	//  戻り値
	int result;
	//  変換結果値
	long _x;
	//  変換値の小数点位置
	int _dp;
	//  符号
	int sign;
	//  変換桁数
	int digit;
	//  有効変換桁数
	int valid;
	//  現在チェック中のキャラクタ
	char _c;

	result = 0;
	//  キャラクタの取得
	_c = **p_str;
	//  符号検出
	if ( _c == '-' ) {
		sign = 1;
		//  次のキャラクタ
		( *p_str )++;
	} else {
		sign = 0;
		if ( _c == '+' ) {
			//  次のキャラクタ
			( *p_str )++;
		}
	}
	//  非小数点部
	_x = 0;
	valid = 0;
	digit = fl_digit( p_str, &_x, &valid );
	//  指数値
	_dp = digit - valid;
	//  キャラクタの取得
	_c = **p_str;
	if ( p_dp ) {
		//  小数点対応
		if ( _c == '.' ) {
			//  '.'で完了時
			//  次のキャラクタ
			( *p_str )++;
			//  小数点部
			valid = 1;
			digit = fl_digit( p_str, &_x, &valid );
			//  指数値が代入されてなければ小数点位置を代入
			if ( _dp == 0 ) {
				_dp = -( valid - 1 );
			}
			//  キャラクタの取得
			_c = **p_str;
		}
		*p_dp = _dp;
	}
	//  結果
	*p_x = sign ? -_x : +_x;
	//  変換終了位置のキャラクタ
	return ( **p_str );
}

/*! 数値を数値文字列へ変換する変換関数（左詰め、正の値のみ、変動桁数）
@param[out]    temp_top 数値文字列への変換結果を格納するテンポラリ領域への先頭ポインタ
@param[in,out] p_point  テンポラリ領域の現在位置が格納されているポインタ
@param[in]     x        変換元となる数値
@return                 変換されきれなかった残りの数値
*/
static long cv_x( char *temp_top, int *p_point, long x )
{
	//  除算結果
	ldiv_t dt;
	//  数字変換
	while ( ( ( *p_point ) > 0 ) && ( x > 0 ) ) {
		( *p_point )--;
		//  最下位数の取得とシフト
		dt = ldiv( x, 10 );
		x = dt.quot;
		//  キャラクタ変換
		temp_top[ *p_point ] = '0' + dt.rem;
	}
	//  値を返す
	return ( x );
}

/*! 数値を数値文字列へ変換する変換関数（左詰め、正の値のみ、固定桁数）
@param[out]    temp_top 数値文字列への変換結果を格納するテンポラリ領域への先頭ポインタ
@param[in,out] p_point  テンポラリ領域の現在位置が格納されているポインタ
@param[in]     x        変換元となる数値
@param[in]     digit    変換する桁数
@return                 変換されきれなかった残りの数値
*/
static long cv_digit( char *temp_top, int *p_point, long x, int digit )
{
	//  除算結果
	ldiv_t dt;
	//  数字変換
	while ( ( ( *p_point ) > 0 ) && ( digit > 0 ) ) {
		( *p_point )--;
		//  最下位数の取得とシフト
		dt = ldiv( x, 10 );
		x = dt.quot;
		//  キャラクタ変換
		temp_top[ *p_point ] = '0' + dt.rem;
		//  桁判定
		digit--;
	}
	//  値を返す
	return ( x );
}

//  テンポラリ領域サイズ
#define  __CV_TEMP_SIZE  ( 24 )  //!<cv()関数で使用するテンポラリ領域のサイズ

/*! 数値を数値文字列へ変換する変換関数（符号対応、小数点位置対応）
@param[out]    str      数値文字列への変換結果を格納するポインタ
@param[in]     x        変換元となる数値
@param[in]     dp       変換する数値の小数点位置
@param[in]     sign     正の値の場合の符号強制
@return                 変換された文字列の長さ
*/
static int cv( char *str, long x, long dp, int sign )
{
	//  テンポラリ領域 - 数字列
	char temp[ __CV_TEMP_SIZE ];
	//  ポイント
	int point;
	//  負の値
	int minus;
	
	//  符号判定
	if ( x < 0 ) {
		minus = 1;
		x = -x;
	} else {
		minus = 0;
	}
	//  終端設定
	point = __CV_TEMP_SIZE - 1;
	temp[ point ] = '\0';
	//  小数点位置判定
	if ( dp < 0 ) {
		//  小数点以下
		x = cv_digit( temp, &point, x, -dp );
		dp = 0;
		//  小数点
		point--;
		temp[ point ] = '.';
	}
	//  仮数値判定
	if ( x > 0 ) {
		//  指数
		cv_digit( temp, &point, 0, dp );
		//  仮数
		cv_x( temp, &point, x );
	} else
	{
		//  ゼロ
		point--;
		temp[ point ] = '0';
	}
	//  符号判定
	if ( minus ) {
		//  マイナス符号
		point--;
		temp[ point ] = '-';
	} else
	if ( sign ) {
		//  プラス符号
		point--;
		temp[ point ] = '+';
	}
	strcpy( str, &temp[ point ] );
	//  文字列の長さを返す
	return ( __CV_TEMP_SIZE - 1 - point );
}

//  テンポラリ領域サイズ
#define  __PKFLT_TO_STR_TEMP_SIZE  ( 24 )  //!<pkflt_to_str_finite()関数で使用するテンポラリ領域のサイズ

/*! PACK_FLOAT型 → 数値文字列 変換関数（有限値限定）
@param[in]      pkflt   変換元となるPACK_FLOAT型浮動小数点値
@param[out]     str     数値文字列への変換結果を格納するポインタ
@return                 渡された str と同じ場所を指すポインタ
@warning                変換元となるPACK_FLOAT型浮動小数点値が有限値でない場合、期待した結果は得られない
*/
static char *pkflt_to_str_finite( PACK_FLOAT pkflt, char *str )
{
	//  テンポラリ領域 - 浮動小数点文字列
	char temp[ __PKFLT_TO_STR_TEMP_SIZE ];
	//  ポイント
	int point;

	//  仮数部 & 小数点位置
	point = cv( &temp[ 0 ], pkflt.kasuu, pkflt.shousuutenn, 0 );
	if ( pkflt.shisuu ) {
		//  指数キャラクタ
		temp[ point ] = 'e';
		point++;
		//  指数部
		cv( &temp[ point ], pkflt.shisuu, 0, 1 );
	}
	//  出力先文字列の先頭位置を返す
	return ( strcpy( str, &temp[ 0 ] ) );
}

/*! 10のべき乗値乗算関数
@param[in]  flt     実数
@param[in]  _e      指数
@return             実数に底を10とした指数のべき乗を掛けた結果
*/
static float pkflt_to_flt_mul_exp( float flt, long _e )
{
	//  乗数
	float m = 10.0f;
	
	for ( ; ; ) {
		if ( _e & 0x00000001 ) {
			flt *= m;
		}
		_e >>= 1;
		if ( !_e ) {
			break;
		}
		m *= m;
	}
	return ( flt );
}

/*! 10のべき乗値除算関数
@param[in]  flt     実数
@param[in]  _e      指数
@return             実数に底を10とした指数のべき乗で割った結果
*/
static float pkflt_to_flt_div_exp( float flt, long _e )
{
	//  乗数
	float m = 10.0f;
	
	for ( ; ; ) {
		if ( _e & 0x00000001 ) {
			flt /= m;
		}
		_e >>= 1;
		if ( !_e ) {
			break;
		}
		m *= m;
	}
	return ( flt );
}

/*! PACK_FLOAT型 → float型 変換関数 （有限値限定）
@param[in]  pkflt   PACK_FLOAT型の値
@return             float型に変換された値
*/
static float pkflt_to_flt_finite( PACK_FLOAT pkflt )
{
	//  戻り値
	float flt = ( float )pkflt.kasuu;
	//  指数部 + 少数点位置
	long _e = pkflt.shisuu + pkflt.shousuutenn;
	
	if ( _e > 0 ) {
		return ( pkflt_to_flt_mul_exp( flt, +_e ) );
	}
	if ( _e < 0 ) {
		return ( pkflt_to_flt_div_exp( flt, -_e ) );
	}
	return ( flt );
}

/*! float型 → PACK_FLOAT型 変換関数 （有限値限定）
@param[in]  flt     float型の値
@return             PACK_FLOAT型に変換された値
*/
static PACK_FLOAT flt_to_pkflt_finite( float flt )
{
	//  結果
	PACK_FLOAT pkflt;
	//  符号
	int sign;
	//  ゼロサプレス量
	int supress;
	//  仮数部
	long _x;
	//  小数点位置
	long _s;
	//  指数部
	long _e;

	//  符号処理
	if ( flt < 0.0f ) {
		flt = -flt;
		sign = 1;
	} else {
		sign = 0;
	}
	//  仮数部
	_e = 0;
#ifdef   CPU_ROUND_ZERO  //  ↓切捨て
	while ( flt < 99999.95f ) {
		flt *= 10.0f;
		_e--;
	}
	while ( flt >= 999999.5f ) {
		//  ※ flt *= 0.1f; では累積誤差が生じる
		flt /= 10.0f;
		_e++;
	}
	_x = ( long )( flt + 0.5f );
#else  //CPU_ROUND_ZERO  //  ↓四捨五入
	while ( flt < 100000.0f ) {
		flt *= 10.0f;
		_e--;
	}
	while ( flt >= 1000000.0f ) {
		//  ※ flt *= 0.1f; では累積誤差が生じる
		flt /= 10.0f;
		_e++;
	}
	_x = ( long )flt;
#endif //CPU_ROUND_ZERO
	supress = 0;
	while ( ( _x % 10 ) == 0 ) {
		_x /= 10;
		supress++;
	}
	//  指数部と小数点位置
	if ( ( _e + supress ) < -6 ) {
		//  9.99999e-99
		_s = supress - 6;
		if ( _s < 0 ) {
			_s++;
		}
		_e += supress - _s;
	} else
	if ( _e <= 0 ) {
		//  0.00001 ~ 999999
		_s = _e + supress;
		_e = 0;
	} else
	{
		//  9.99999e+99
		_s = supress - 5;
		_e += 5;
	}
	//  パッケージ
	pkflt.shisuu = _e;
	pkflt.shousuutenn = _s;
	pkflt.kasuu = sign ? -_x : +_x;
	return ( pkflt );
}

/******************************************************************************
  グローバル関数
******************************************************************************/

/*! PACK_FLOAT型を作成する関数
@param[in]      _kasuu          仮数部の値
@param[in]      _shousuutenn    小数点位置の値
@param[in]      _shisuu         指数部の値
@return                         PACK_FLOAT型の値
*/
PACK_FLOAT make_pkflt( long _kasuu, long _shousuutenn, long _shisuu )
{
	PACK_FLOAT pkflt;

	//  小数点値指数化処理
	if ( ( _shousuutenn < -8 ) || ( 7 < _shousuutenn ) ) {
		_shisuu += ( _shousuutenn - -5 );
		_shousuutenn = -5;
	}
	//  指数判定
	if ( _shisuu < -64 ) {
		//  小さい値
		pkflt = PACK_FLOAT_ZERO;
	} else
	if ( _shisuu > +63 ) {
		//  大きい値
		if ( _kasuu > 0 ) {
			//  正の無限大
			pkflt = PACK_FLOAT_PINF;
		} else
		if ( _kasuu < 0 ) {
			//  負の無限大
			pkflt = PACK_FLOAT_NINF;
		} else
		{
			//  ゼロ
			pkflt = PACK_FLOAT_ZERO;
		}
	} else
	{
		//  通常値
		pkflt.kasuu       = _kasuu;
		pkflt.shousuutenn = _shousuutenn;
		pkflt.shisuu      = _shisuu;
	}

	return ( pkflt );
}

/*! 数値文字列 → PACK_FLOAT型 変換関数
@param[in]      str     変換元となる数値文字列の格納されているポインタ
@param[out]     p_pkflt PACK_FLOAT型浮動小数点値への変換結果を格納するポインタ
@retval         0       正常終了
@retval         1       数値文字列の文法エラー（変換未実行）
@attention              数値文字列 → PACK_FLOAT型 の変換は縮小変換です。以下の場合、情報を失う事があります。
@attention              ・先頭の 0 は無視されます。（"0001" → 1）
@attention              ・変換の際に有効数字10進数6桁に丸められます。（"3.14159265358979" → 3.14159、369258147 → 369258000）
@attention              ・14桁以上の大きな値は小数点位置から指数部への置き換えとなります。（"1000000000000000000000000000000000" → 1.00000e+33）
@attention              ・小数点9桁以下の小さな値は小数点位置から指数部への置き換えとなります。（"0.00000000000000000000000000000001" → 0.00001e-27）
@warning                ・無限大、および非数を示す文字列からの変換には対応していません。（"#NAN" → 文法エラー）
*/
int str_to_pkflt( const char *str, PACK_FLOAT *p_pkflt )
{
	long _kasuu;
	long _shisuu;
	int _shousuutenn;
	//  現在チェック中のキャラクタ
	char _c;

	//  仮数部
	_c = fl( &str, &_kasuu, &_shousuutenn );
	if ( _c == '\0' ) {
		//  キャラクタなし（'\0'で終了）
		//  指数部 = 0
		_shisuu = 0;
	} else
	if ( _c == 'e' || _c == 'E' ) {
		//  キャラクタが指数（'e'or'E'）
		str++;
		//  指数部
		if ( fl( &str, &_shisuu, NULL ) ) {
			//  '\0'以外で終了
			//  文法エラー
			return ( 1 );
		}
	} else
	{
		//  キャラクタがそれ以外
		//  文法エラー
		return ( 1 );
	}
	//  結果の格納
	*p_pkflt = make_pkflt( _kasuu, _shousuutenn, _shisuu );
	//  正常終了
	return ( 0 );
}

/*! PACK_FLOAT型 → 数値文字列 変換関数
@param[in]      pkflt   変換元となるPACK_FLOAT型浮動小数点値
@param[out]     str     数値文字列への変換結果を格納するポインタ
@return                 渡された str と同じ場所を指すポインタ
@note                   PACK_FLOAT型 → 数値文字列 の変換は拡大変換です。情報を失う事はありません。
@note                   以下の場合、特殊文字列に変換します。
@note                   ・正の無限大の場合、変換結果の文字列は "+#INF"
@note                   ・負の無限大の場合、変換結果の文字列は "-#INF"
@note                   ・非数の場合、変換結果の文字列は "#NAN"
*/
char *pkflt_to_str( PACK_FLOAT pkflt, char *str )
{
	//  仮数部
	long _x = pkflt.kasuu;
	
	//  有限値判定
	if ( ( -999999 <= _x ) && ( _x <= +999999 ) ) {
		return ( pkflt_to_str_finite( pkflt, str ) );
	}
	//  正の無限大判定
	if ( _x == PACK_FLOAT_KASUU_PINF ) {
		return ( strcpy( str, "+#INF" ) );
	}
	//  負の無限大判定
	if ( _x == PACK_FLOAT_KASUU_NINF ) {
		return ( strcpy( str, "-#INF" ) );
	}
	//  非数(NaN)
	return ( strcpy( str, "#NAN" ) );
}

/*! PACK_FLOAT型 → float型 変換関数
@param[in]      pkflt   PACK_FLOAT型の値
@return                 float型に変換された値
@note                   ・PACK_FLOAT型の値が正の無限大の場合、変換結果は正の無限大を示すfloat型の値へ変換されます。
@note                   ・PACK_FLOAT型の値が負の無限大の場合、変換結果は負の無限大を示すfloat型の値へ変換されます。
@note                   ・PACK_FLOAT型の値が非数の場合、変換結果は非数を示すfloat型の値へ変換されます。
@attention              PACK_FLOAT型 → float型 の変換は縮小変換です。以下の場合、情報を失う事があります。
@attention              ・PACK_FLOAT型の持つ小数点位置と指数部の値は、変換の際に統合されるため保持されません。（1.024e+3 → 1024.0f）
@attention              ・PACK_FLOAT型の持つ値がfloat型で表せない小さな値（※1）だった場合、変換結果はゼロを示すfloat型の値へ変換されます。（-9.15527e-50 → -0.0f）
@attention              （※2）・PACK_FLOAT型の持つ値がfloat型で表せない大きな値（※1）だった場合、変換結果は無限大を示すfloat型の値へ変換されます。（8.97984e+38 → 正の無限大）
@attention              ※1：float型で表せる範囲は使用する開発環境に依存します。
@attention              ※2：無限大に変換されるか否かは使用するCPUに依存します。
@note                   HEW（SH2-FPU）の場合、1.175494e-38 ～ 3.402823e+38、無限大への拡張無し（最大値で制限）
@note                   Visual C++ の場合、1.401298e-45 ～ 3.402823e+38、無限大への拡張有り
*/
float pkflt_to_flt( PACK_FLOAT pkflt )
{
	//  仮数部
	long _x = pkflt.kasuu;
	
	//  ゼロ判定
	if ( _x == 0 ) {
		return ( 0.0f );
	}
	//  有限値判定
	if ( ( -999999 <= _x ) && ( _x <= +999999 ) ) {
		return ( pkflt_to_flt_finite( pkflt ) );
	}
	//  正の無限大判定
	if ( _x == PACK_FLOAT_KASUU_PINF ) {
		return ( +INFINITYF );
	}
	//  負の無限大判定
	if ( _x == PACK_FLOAT_KASUU_NINF ) {
		return ( -INFINITYF );
	}
	//  非数(NaN)
	return ( NANF );
}

/*! float型 → PACK_FLOAT型 変換関数
@param[in]      flt     float型の値
@return                 PACK_FLOAT型に変換された値
@note                   ・float型の値が正の無限大の場合、変換結果は正の無限大を示すPACK_FLOAT型の値へ変換されます。
@note                   ・float型の値が負の無限大の場合、変換結果は負の無限大を示すPACK_FLOAT型の値へ変換されます。
@note                   ・float型の値が非数の場合、変換結果は非数を示すPACK_FLOAT型の値へ変換されます。
@attention              float型 → PACK_FLOAT型 の変換は縮小変換です。以下の場合、情報を失う事があります。
@attention              ・変換の際に有効数字10進数6桁に丸められます。（3.14159265358979f → 3.14159）
*/
PACK_FLOAT flt_to_pkflt( float flt )
{
	int  res;

	//  非数(NaN)判定
	if ( isnanf( flt ) ) {
		return ( PACK_FLOAT_NAN );
	}
	res = isinff( flt );
	//  正の無限大判定
	if ( res > 0 ) {
		return ( PACK_FLOAT_PINF );
	}
	//  負の無限大判定
	if ( res < 0 ) {
		return ( PACK_FLOAT_NINF );
	}
	//  ゼロ判定
	if ( flt == 0.0f ) {
		return ( _pkflt_zero.pack );
	}
	//  有限値
	return ( flt_to_pkflt_finite( flt ) );
}

/*****************************************************************************/

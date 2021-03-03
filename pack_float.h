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
@file           pack_float.h
*******************************************************************************
@brief          PACK_FLOAT ライブラリ（ヘッダ）
*******************************************************************************
@date           2016.01.20
@author         H.SHINGA
@note           ・HEW（Renesas）および Visual C++（Microsoft）コンパイラへの対応。
@note           ・無限大および非数に対応する様に変更。判定関数の追加。
@note           ・関数の複雑度に対する改善。（関数の分散化）
@note           ・エンディアン構成判定を追加
@note           ・HEW の bit_order オプションに対応
@note           ・コメント文のDoxygen対応
*******************************************************************************
@date           2016.01.21
@author         H.SHINGA
@note           ・コメント文追記
******************************************************************************/

#ifndef PACK_FLOAT_INCLUDED
#define PACK_FLOAT_INCLUDED

/******************************************************************************
  構成定義
******************************************************************************/

//  エンディアン構成

#ifndef   __BIG_ENDIAN

//  HEW
#ifdef    __HITACHI__

#ifdef    _LIT

//  _LIT がプリデファインドされている場合はリトルエンディアン

#else   //_LIT

#define   __BIG_ENDIAN  ( 1 )  //!<ビッグエンディアンである事を示す構成定義

#endif  //_LIT

#endif  //__HITACHI__

//  Visual C++
#ifdef    _MSC_VER

//  Visual C++ はリトルエンディアン

#endif  //_MSC_VER

#endif  //__BIG_ENDIAN

/******************************************************************************
  型定義
******************************************************************************/

#ifdef    __BIG_ENDIAN
//  ビッグエンディアン

//  HEW
#ifdef    __HITACHI__
//  ビットフィールド境界を4バイトにする
#pragma  pack  4
//  ビットフィールドを上位詰めにする
#pragma  bit_order  left
#endif  //__HITACHI__

/*! PACK_FLOAT型構造体
*/
typedef struct _pack_float {
#ifdef __GNUC__
	//  MSB
	long  kasuu        : 21;  //!<仮数部     (-1048576 ～ 0 ～ 1048575)
	long  shousuutenn  :  4;  //!<少数点位置 (-8 ～ 0 ～ 7)
	long  shisuu       :  7;  //!<指数部     (-64 ～ 0 ～ 63)
	//  LSB
} PACK_FLOAT;
#else
	//  MSB
	long  shisuu       :  7;  //!<指数部     (-64 ～ 0 ～ 63)
	long  shousuutenn  :  4;  //!<少数点位置 (-8 ～ 0 ～ 7)
	long  kasuu        : 21;  //!<仮数部     (-1048576 ～ 0 ～ 1048575)
	//  LSB
} PACK_FLOAT;
#endif

/*! long型 - PACK_FLOAT型 32bit共用体
*/
typedef union _pack_float_dual {
	long        value;  //!<long型数値
	PACK_FLOAT  pack;   //!<PACK_FLOAT型構造体
} PACK_FLOAT_DUAL;

//  HEW
#ifdef    __HITACHI__
//  ビットフィールド境界サイズ指定を終了する
#pragma  unpack
//  ビットフィールドを標準設定にする
#pragma  bit_order
#endif  //__HITACHI__

#else   //__BIG_ENDIAN
//  リトルエンディアン

//  HEW
#ifdef    __HITACHI__
//  ビットフィールド境界を4バイトにする
#pragma  pack  4
//  ビットフィールドを下位詰めにする
#pragma  bit_order  right
#endif  //__HITACHI__

/*! PACK_FLOAT型構造体
*/
typedef struct _pack_float {
	//  LSB
	long  kasuu        : 21;  //!<仮数部     (-1048576 ～ 0 ～ 1048575)
	long  shousuutenn  :  4;  //!<少数点位置 (-8 ～ 0 ～ 7)
	long  shisuu       :  7;  //!<指数部     (-64 ～ 0 ～ 63)
	//  MSB
} PACK_FLOAT;

/*! long型 - PACK_FLOAT型 32bit共用体
*/
typedef union _pack_float_dual {
	long        value;  //!<long型数値
	PACK_FLOAT  pack;   //!<PACK_FLOAT型構造体
} PACK_FLOAT_DUAL;

//  HEW
#ifdef    __HITACHI__
//  ビットフィールド境界サイズ指定を終了する
#pragma  unpack
//  ビットフィールドを標準設定にする
#pragma  bit_order
#endif  //__HITACHI__

#endif  //__BIG_ENDIAN

/******************************************************************************
  定数定義
******************************************************************************/

//  PACK_FLOAT型の定数定義

/*!
@def        PACK_FLOAT_ZERO
@warning    この定数は代入のみに使用し、比較演算には使用しないこと。
*/
#define   PACK_FLOAT_ZERO  ( _pkflt_zero.pack )  //!<PACK_FLOAT型、ゼロ定数定義

/*!
@def        PACK_FLOAT_PINF
@warning    この定数は代入のみに使用し、比較演算には使用しないこと。
*/
#define   PACK_FLOAT_PINF  ( _pkflt_pinf.pack )  //!<PACK_FLOAT型、正の無限大定数定義

/*!
@def        PACK_FLOAT_NINF
@warning    この定数は代入のみに使用し、比較演算には使用しないこと。
*/
#define   PACK_FLOAT_NINF  ( _pkflt_ninf.pack )  //!<PACK_FLOAT型、負の無限大定数定義

/*!
@def        PACK_FLOAT_NAN
@warning    この定数は代入のみに使用し、比較演算には使用しないこと。
*/
#define   PACK_FLOAT_NAN  ( _pkflt_nan.pack )  //!<PACK_FLOAT型、非数定数定義

//  PACK_FLOAT型仮数部の定数定義

/*!
@def        PACK_FLOAT_KASUU_PINF
@warning    比較演算で使用する場合、対象はPACK_FLOAT型構造体の仮数部のみとすること。
*/
#define   PACK_FLOAT_KASUU_PINF  ( 0x000FFFFF )  //!<PACK_FLOAT型仮数部、正の無限大定数定義

/*!
@def        PACK_FLOAT_KASUU_NINF
@warning    比較演算で使用する場合、対象はPACK_FLOAT型構造体の仮数部のみとすること。
*/
#define   PACK_FLOAT_KASUU_NINF  ( 0xFFF00000 )  //!<PACK_FLOAT型仮数部、負の無限大定数定義

/******************************************************************************
  C / C++ 定義
******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif
	
/******************************************************************************
  グローバル定数定義
******************************************************************************/

//  PACK_FLOAT型の定数

extern PACK_FLOAT_DUAL const  _pkflt_zero;  //!<PACK_FLOAT型、ゼロ定数

extern PACK_FLOAT_DUAL const  _pkflt_pinf;  //!<PACK_FLOAT型、正の無限大定数

extern PACK_FLOAT_DUAL const  _pkflt_ninf;  //!<PACK_FLOAT型、負の無限大定数

extern PACK_FLOAT_DUAL const  _pkflt_nan;   //!<PACK_FLOAT型、非数定数

/******************************************************************************
  ローカル関数宣言
******************************************************************************/

#ifdef  __HITACHI__
#pragma  inline  ( pkflt_isinf )
#endif
static int pkflt_isinf( PACK_FLOAT pkflt );

#ifdef  __HITACHI__
#pragma  inline  ( pkflt_isnan )
#endif
static int pkflt_isnan( PACK_FLOAT pkflt );

#ifdef  __HITACHI__
#pragma  inline  ( pkflt_isfinite )
#endif
static int pkflt_isfinite( PACK_FLOAT pkflt );

/******************************************************************************
  ローカル関数
******************************************************************************/

/*!無限大判定関数 (PACK_FLOAT型)
@param[in]  pkflt               判定対象となる浮動小数点値
@retval     正の整数値(1以上)   正の無限大である
@retval     負の整数値          負の無限大である
@retval     0                   無限大ではない
*/
static int pkflt_isinf( PACK_FLOAT pkflt )
{
	//  仮数部
	long _x = pkflt.kasuu;
	//  正の無限大判定
	if ( _x == PACK_FLOAT_KASUU_PINF ) {
		return ( +1 );
	}
	//  負の無限大判定
	if ( _x == PACK_FLOAT_KASUU_NINF ) {
		return ( -1 );
	}
	//  無限大ではない
	return ( 0 );
}

/*!非数判定関数 (PACK_FLOAT型)
@param[in]  pkflt   判定対象となる浮動小数点値
@retval     0以外   非数(NaN)である
@retval     0       非数(NaN)ではない
*/
static int pkflt_isnan( PACK_FLOAT pkflt )
{
	//  仮数部
	long _x = pkflt.kasuu;
	//  有限値判定
	if ( ( -999999 <= _x ) && ( _x <= +999999 ) ) {
		return ( 0 );
	}
	//  正の無限大判定
	if ( _x == PACK_FLOAT_KASUU_PINF ) {
		return ( 0 );
	}
	//  負の無限大判定
	if ( _x == PACK_FLOAT_KASUU_NINF ) {
		return ( 0 );
	}
	//  非数(NaN)である
	return ( 1 );
}

/*!有限値判定関数 (PACK_FLOAT型)
@param[in]  pkflt   判定対象となる浮動小数点値
@retval     0以外   有限値である（無限大または非数(NaN)ではない）
@retval     0       有限値ではない（無限大または非数(NaN)のいずれかである）
*/
static int pkflt_isfinite( PACK_FLOAT pkflt )
{
	//  仮数部
	long _x = pkflt.kasuu;
	//  有限値判定
	if ( ( -999999 <= _x ) && ( _x <= +999999 ) ) {
		return ( 1 );
	}
	//  有限値ではない
	return ( 0 );
}

/******************************************************************************
  グローバル関数宣言
******************************************************************************/

/*  PACK_FLOAT型を作成する関数
@param[in]      _kasuu          仮数部の値
@param[in]      _shousuutenn    小数点位置の値
@param[in]      _shisuu         指数部の値
@return                         PACK_FLOAT型の値
*/
PACK_FLOAT make_pkflt( long _kasuu, long _shousuutenn, long _shisuu );

/*  数値文字列 → PACK_FLOAT型 変換関数
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
int str_to_pkflt( const char *str, PACK_FLOAT *p_pkflt );

/*  PACK_FLOAT型 → 数値文字列 変換関数
@param[in]      pkflt   変換元となるPACK_FLOAT型浮動小数点値
@param[out]     str     数値文字列への変換結果を格納するポインタ
@return                 渡された str と同じ場所を指すポインタ
@note                   PACK_FLOAT型 → 数値文字列 の変換は拡大変換です。情報を失う事はありません。
@note                   以下の場合、特殊文字列に変換します。
@note                   ・正の無限大の場合、変換結果の文字列は "+#INF"
@note                   ・負の無限大の場合、変換結果の文字列は "-#INF"
@note                   ・非数の場合、変換結果の文字列は "#NAN"
*/
char *pkflt_to_str( PACK_FLOAT pkflt, char *str );

/*  PACK_FLOAT型 → float型 変換関数
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
float pkflt_to_flt( PACK_FLOAT pkflt );

/*  float型 → PACK_FLOAT型 変換関数
@param[in]      flt     float型の値
@return                 PACK_FLOAT型に変換された値
@note                   ・float型の値が正の無限大の場合、変換結果は正の無限大を示すPACK_FLOAT型の値へ変換されます。
@note                   ・float型の値が負の無限大の場合、変換結果は負の無限大を示すPACK_FLOAT型の値へ変換されます。
@note                   ・float型の値が非数の場合、変換結果は非数を示すPACK_FLOAT型の値へ変換されます。
@attention              float型 → PACK_FLOAT型 の変換は縮小変換です。以下の場合、情報を失う事があります。
@attention              ・変換の際に有効数字10進数6桁に丸められます。（3.14159265358979f → 3.14159）
*/
PACK_FLOAT flt_to_pkflt( float flt );

/******************************************************************************
  C / C++ 定義
******************************************************************************/

#ifdef __cplusplus
}
#endif

/*****************************************************************************/

#endif /* PACK_FLOAT_INCLUDED */

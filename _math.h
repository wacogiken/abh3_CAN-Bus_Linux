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
@file           _math.h
*******************************************************************************
@brief          浮動小数点ライブラリ
*******************************************************************************
@version        1.0.0
@date           2016.01.07
@author         H.SHINGA (Waco Giken Co., Ltd.)
*******************************************************************************
@version        1.1.0
@date           2016.01.08
@author         H.SHINGA (Waco Giken Co., Ltd.)
@note           有限値判定関数を追加
*******************************************************************************
@version        1.2.0
@date           2016.01.19
@author         H.SHINGA (Waco Giken Co., Ltd.)
@note           無限および非数定数定義を追加
*******************************************************************************
@version        1.2.1
@date           2016.01.21
@author         H.SHINGA (Waco Giken Co., Ltd.)
@note           コメント文を追加
*******************************************************************************
@version        1.2.2
@date           2016.01.22
@author         H.SHINGA (Waco Giken Co., Ltd.)
@note           HEWで用意されていた非数定数がC++コンパイラ環境では使用できなかったため、独自定数へ変更
@note           非数定数をマルチエンディアン対応値に変更
******************************************************************************/

#ifndef  _INCLUDED__MATH_H_
#define  _INCLUDED__MATH_H_

/******************************************************************************
  インクルードファイル
******************************************************************************/

#include <math.h>
//  HEW で使用する場合、mathf.h をインクルードする
#ifdef  __HITACHI__
#include <mathf.h>
#endif

#include <float.h>

/******************************************************************************
  定数定義
******************************************************************************/

//  無限大定数の定義

//  HEW
#ifdef    __HITACHI__

#ifndef   INFINITY
#define   INFINITY   HUGE_VAL   //!<正の無限大を示すdouble型の定数定義
#endif  //INFINITY

#ifndef   INFINITYF
#define   INFINITYF  HUGE_VALF  //!<正の無限大を示すfloat型の定数定義
#endif  //INFINITYF

#endif  //__HITACHI__

//  Visual C++
#ifdef    _MSC_VER

#ifndef   INFINITY
#define   INFINITY   HUGE_VAL   //!<正の無限大を示すdouble型の定数定義
#endif  //INFINITY

#ifndef   INFINITYF
#define   INFINITYF  ( ( float )INFINITY )  //!<正の無限大を示すfloat型の定数定義
#endif  //INFINITYF

#endif  //_MSC_VER

#ifdef __GNUC__
#define   INFINITYF  ( ( float )INFINITY )  //!<正の無限大を示すfloat型の定数定義
#endif

//  非数定数の定義

static const unsigned long _c_nan[ 2 ] = { 0x7FFFFFFF, 0x7FFFFFFF };  //!<非数を示す定数

/*!
@def        NAN
@warning    この定数は代入のみに使用し、比較演算には使用しないこと。
*/
#ifndef   NAN
#define   NAN  ( *( double * )_c_nan )  //!<非数を示すdouble型の定数定義
#endif  //NAN

/*!
@def        NANF
@warning    この定数は代入のみに使用し、比較演算には使用しないこと。
*/
#ifndef   NANF
#define   NANF  ( *( float * )_c_nan )  //!<非数を示すfloat型の定数定義
#endif  //NANF

/******************************************************************************
  C / C++ 定義
******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
  関数宣言
******************************************************************************/

//  HEW で使用する場合、インライン化する

#ifdef  __HITACHI__
#pragma  inline  ( isinf )
#endif
#ifndef __GNUC__
static int isinf( double x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isnan )
#endif
#ifndef __GNUC__
static int isnan( double x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isfinite )
#endif
#ifndef __GNUC__
static int isfinite( double x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isnormal )
#endif
#ifndef __GNUC__
static int isnormal( double x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isinff )
#endif
#ifndef __GNUC__
static int isinff( float x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isnanf )
#endif
#ifndef __GNUC__
static int isnanf( float x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isfinitef )
#endif
#ifndef __GNUC__
static int isfinitef( float x );
#endif

#ifdef  __HITACHI__
#pragma  inline  ( isnormalf )
#endif
static int isnormalf( float x );

/******************************************************************************
  関数
******************************************************************************/

/*!無限大判定関数 (double型)
@param[in]  x                   判定対象となる浮動小数点値
@retval     正の整数値(1以上)   正の無限大である
@retval     負の整数値          負の無限大である
@retval     0                   無限大ではない
*/
#ifndef __GNUC__
static int isinf( double x )
{
	if ( x == +INFINITY ) {
		return ( +1 );
	}
	if ( x == -INFINITY ) {
		return ( -1 );
	}
	return ( 0 );
}
#endif

/*!非数判定関数 (double型)
@param[in]  x       判定対象となる浮動小数点値
@retval     0以外   非数(NaN)である
@retval     0       非数(NaN)ではない
*/
#ifndef __GNUC__
static int isnan( double x )
{
	return ( x != x );
}
#endif

/*!有限値判定関数 (double型)
@param[in]  x       判定対象となる浮動小数点値
@retval     0以外   有限値である（無限大または非数(NaN)ではない）
@retval     0       有限値ではない（無限大または非数(NaN)のいずれかである）
*/
#ifndef __GNUC__
static int isfinite( double x )
{
	if ( x == +INFINITY ) {
		return ( 0 );
	}
	if ( x == -INFINITY ) {
		return ( 0 );
	}
	return ( x == x );
}
#endif

/*!通常の浮動小数点値判定関数 (double型)
@param[in]  x       判定対象となる浮動小数点値
@retval     0以外   通常の浮動小数点値である（無限大または非数(NaN)ではない）
@retval     0       通常の浮動小数点値ではない（無限大または非数(NaN)のいずれかである）
*/
#ifndef __GNUC__
static int isnormal( double x )
{
	if ( x == +INFINITY ) {
		return ( 0 );
	}
	if ( x == -INFINITY ) {
		return ( 0 );
	}
	return ( x == x );
}
#endif

/*!無限大判定関数 (float型)
@param[in]  x                   判定対象となる浮動小数点値
@retval     正の整数値(1以上)   正の無限大である
@retval     負の整数値          負の無限大である
@retval     0                   無限大ではない
*/
#ifndef __GNUC__
static int isinff( float x )
{
	if ( x == +INFINITYF ) {
		return ( +1 );
	}
	if ( x == -INFINITYF ) {
		return ( -1 );
	}
	return ( 0 );
}
#endif

/*!非数判定関数 (float型)
@param[in]  x       判定対象となる浮動小数点値
@retval     0以外   非数(NaN)である
@retval     0       非数(NaN)ではない
*/
#ifndef __GNUC__
static int isnanf( float x )
{
	return ( x != x );
}
#endif

/*!有限値判定関数 (float型)
@param[in]  x       判定対象となる浮動小数点値
@retval     0以外   有限値である（無限大または非数(NaN)ではない）
@retval     0       有限値ではない（無限大または非数(NaN)のいずれかである）
*/
#ifndef __GNUC__
static int isfinitef( float x )
{
	if ( x == +INFINITYF ) {
		return ( 0 );
	}
	if ( x == -INFINITYF ) {
		return ( 0 );
	}
	return ( x == x );
}
#endif

/*!通常の浮動小数点値判定関数 (float型)
@param[in]  x       判定対象となる浮動小数点値
@retval     0以外   通常の浮動小数点値である（無限大または非数(NaN)ではない）
@retval     0       通常の浮動小数点値ではない（無限大または非数(NaN)のいずれかである）
*/
static int isnormalf( float x )
{
	if ( x == +INFINITYF ) {
		return ( 0 );
	}
	if ( x == -INFINITYF ) {
		return ( 0 );
	}
	return ( x == x );
}

/******************************************************************************
  C / C++ 定義
******************************************************************************/

#ifdef __cplusplus
}
#endif

/*****************************************************************************/

#endif

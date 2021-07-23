/******************************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Waco Giken Co., Ltd.
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
@file           canABH3.h
*******************************************************************************
@brief          ABH3用CAN サンプルソフト（ドライバ・ヘッダ）
*******************************************************************************
@date           2021.02.26
@author         T.Furusawa
@note           ・初版

@date           2021.06.11
@author         T.Furusawa
@note           ・V1.2 シングルパケット1に関してコメントアウト
@note           ・     ブロードキャストパケットのブロードキャストリクエストを追加
******************************************************************************/

#ifndef CAN_ABH3_INCLUDED
#define CAN_ABH3_INCLUDED

/******************************************************************************
  インクルードファイル
******************************************************************************/

#include <inttypes.h>
#include <linux/can.h>
#include <net/if.h>

/******************************************************************************
  型定義
******************************************************************************/
/*! canABH3型構造体
*/
#pragma pack(push)  /* 現在のアライメントをスタックにプッシュ */
#pragma pack(1)     /* 1バイト境界にアライメントを設定 */
typedef struct _canABH3 {
	short   abh3ID;     //!<ABH3アドレス      (0 ～ 253)
	short   hostID;     //!<ホスト機器アドレス (0 ～ 253)
    short   broadGroup; //!<ブロードキャスト送信グループ番号 (0～32)
    short   priority;   //!<共通プライオリティ (0～7)
    short   timeOut;    //!<タイムアウト　　　 (0 ～ 10000)
	unsigned char   pgn;    //!<シングルパケットPGN (0 ～ 255)
    char *device;       //!<デバイス名称
    struct {
        long singleDP0Send; // シングルパケット DP0 送信
        long singleDP0Recv; // シングルパケット DP0 受信
        long singleDP1Send; // シングルパケット DP1 送信
        long singleDP1Recv; // シングルパケット DP1 受信
        long broadReqSend;  // ブロードキャストリクエスト 送信
        long broadBaseRecv; // ブロードキャスト 受信
        long multiRecv;     // マルチキャストPGN無し 受信
        long multiCtrlSend; // マルチキャスト制御 送信
        long multiCtrlRecv; // マルチキャスト制御 受信
        long multiDataSend; // マルチキャストデータ 送信
        long multiDataRecv; // マルチキャストデータ 受信
    } id;
    struct {
        int socket;
        struct can_frame frame;
        fd_set rdfs;
    } can;
    struct {
        int16_t cmdAY;
        int16_t cmdBX;
        int32_t input;
    } singleDP0;
} CAN_ABH3;

#pragma pack(1)
typedef union _canABH3Data {
    struct {
        uint8_t buf[8];
    } raw;
    union {
        struct {
            int16_t AY;
            int16_t BX;
            int32_t input;
        } cmd;
        struct {
            int16_t A;
            int16_t B;
            int16_t Y;
            int16_t X;
        } fbk;
    } singleDP0;
    // 保留
    /*
    struct {
        int32_t  pulseA;
        int32_t  pulseB;
    } singleDP1;
    */
    struct {
        int32_t error;
        int32_t alarm;
    } broad0;
    struct {
        int32_t control;
        int32_t in_out;
    } broad1;
    struct {
        int16_t velCmdAY;
        int16_t velCmdBX;
        int16_t velFbkAY;
        int16_t velFbkBX;
    } broad2;
    struct {
        int16_t curCmdAY;
        int16_t curCmdBX;
        int16_t loadA;
        int16_t loadB;
    } broad3;
    struct {
        int32_t pulseA;
        int32_t pulseB;
    } broad4;
    struct {
        int16_t analog0;
        int16_t analog1;
        int16_t mainVolt;
        int16_t controlVolt;
    } broad5;
    struct {
        float monitor0;
        float monitor1;
    } broad6;
} CAN_ABH3_DATA;
#pragma pack(pop)   /* スタックから元のアライメントを復元 */

/******************************************************************************
  定数定義
******************************************************************************/

//  変換係数の定義

#define UNIT_VEL    (0.2)   // 速度
#define UNIT_CUR    (0.01)  // 電流
#define UNIT_LOAD   (0.1)   // 負荷率
#define UNIT_ANALOG (0.01)  // アナログ
#define UNIT_VOLT   (0.1)   // 電圧

/******************************************************************************
  ローカル関数宣言
******************************************************************************/

/******************************************************************************
  グローバル関数宣言
******************************************************************************/

/*  速度データの浮動小数点形式からCAN形式への変換
@param[in]      vel         浮動小数点で表した単位[min-1]の速度データ
@return                     CANで使用する、整数で表した単位[0.2min-1]の速度データ　-6553.6[min-1]～0[min-1]～6553.4[min-1]
*/
short cnvVel2CAN(float vel);

/*  速度データのCAN形式から浮動小数点形式への変換
@param[in]      vel         CANで使用する、整数で表した単位[0.2min-1]の速度データ　-6553.6[min-1]～0[min-1]～6553.4[min-1]
@return                     浮動小数点で表した単位[min-1]の速度データ
*/
float cnvCAN2Vel(short vel);

/*  電流データの浮動小数点形式からCAN形式への変換
@param[in]      cur         浮動小数点で表した単位[%]の電流データ
@return                     CANで使用する、整数で表した単位[0.01%]の電流データ　-327.68[%]～0.00[%]～327.67[%]
*/
short cnvCur2CAN(float cur);

/*  電流データのCAN形式から浮動小数点形式への変換
@param[in]      cur         CANで使用する、整数で表した単位[0.01%]の電流データ　-327.68[%]～0.00[%]～327.67[%]
@return                     浮動小数点で表した単位[%]の電流データ
*/
float cnvCAN2Cur(short cur);

/*  負荷率データのCAN形式から浮動小数点形式への変換
@param[in]      load        CANで使用する、整数で表した単位[1%]の負荷率データ　0.0[%]～3276.7[%]
@return                     浮動小数点で表した単位[%]の負荷率データ
*/
float cnvCAN2Load(short load);

/*  アナログ入力データのCAN形式から浮動小数点形式への変換
@param[in]      analog      CANで使用する、整数で表した単位[0.01V]のアナログ入力データ　-327.68[V]～0.00[V]～327.67[V]
@return                     浮動小数点で表した単位[V]のアナログ入力データ
*/
float cnvCAN2Analog(short analog);

/*  電源電圧データのCAN形式から浮動小数点形式への変換
@param[in]      volt        CANで使用する、整数で表した単位[0.01V]の電源電圧データ　0.0[V]～3276.7[V]
@return                     浮動小数点で表した単位[V]の電源電圧データ
*/
float cnvCAN2Volt(short volt);

/*  CANと指令の初期化
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_init(CAN_ABH3 *abh3Ptr);

/*  CANの初期化
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      device      CANデバイス名称
@param[in]      abh3ID      ABH3のID
@param[in]      hostID      ホスト機器のID (自身のID)
@param[in]      priority    パケットの優先度 (共通)
@param[in]      broadGroup  ブロードキャストグループ番号
@param[in]      timeOut     受信タイムアウト時間[ms]
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_port_init(CAN_ABH3 *abh3Ptr, char *device, int abh3ID, int hostID, int priority, int broadGroup, int timeOut);

/*  指令の初期化
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_cmd_init(CAN_ABH3 *abh3Ptr);

/*  CANを閉じる
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_finish(CAN_ABH3 *abh3Ptr);

/*  指令の送信（軸別）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      cmd         指令値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_cmdAY(CAN_ABH3 *abh3Ptr, short cmd, CAN_ABH3_DATA *ptr);
int abh3_can_cmdBX(CAN_ABH3 *abh3Ptr, short cmd, CAN_ABH3_DATA *ptr);

/*  指令の送信（同時）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      cmdAY       A/Y指令値
@param[in]      cmdBX       B/X指令値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_cmd(CAN_ABH3 *abh3Ptr, short cmdAY, short cmdBX, CAN_ABH3_DATA *ptr);

/*  入力の送信（一括）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      data        データ値
@param[in]      mask        マスク値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_inSet(CAN_ABH3 *abh3Ptr, long data, long mask, CAN_ABH3_DATA *ptr);

/*  入力の送信（ビット）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      num         ビット番号(0～31)
@param[in]      data        設定データ(0～1)
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_inBitSet(CAN_ABH3 *abh3Ptr, char num, char data, CAN_ABH3_DATA *ptr);

/*  指令と入力の送信
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      cmdAY       A/Y指令値
@param[in]      cmdBX       B/X指令値
@param[in]      data        データ値
@param[in]      mask        マスク値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_cmdAll(CAN_ABH3 *abh3Ptr, short cmdAY, short cmdBX, long data, long mask, CAN_ABH3_DATA *ptr);

// 保留
/*  積算値のリクエスト
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
// int abh3_can_reqPulse(CAN_ABH3 *abh3Ptr, CAN_ABH3_DATA *ptr);

/*  ブロードキャストパケットのリクエスト
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      num         番号(0x00～0xff)
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_reqBRD(CAN_ABH3 *abh3Ptr, int num, CAN_ABH3_DATA *ptr);
int abh3_can_reqBRDBRD(CAN_ABH3 *abh3Ptr, int num, CAN_ABH3_DATA *ptr);

/*  シングルパケットのリクエスト
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      num         番号(0x00～0x01)
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_reqSNG(CAN_ABH3 *abh3Ptr, int num, CAN_ABH3_DATA *ptr);

/*  マルチパケットによるTelABH3パケットの送受信
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      sbuf        送信文字列へのポインタ(TelABH3プロトコル受信データフォーマット)
@param[in]      rbuf        受信文字列へのポインタ(TelABH3プロトコル送信データフォーマット)
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_trans(CAN_ABH3 *abh3Ptr, char *sbuf, char *rbuf);

#endif /* CAN_ABH3_INCLUDED */

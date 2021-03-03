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
@file           canABH3.c
*******************************************************************************
@brief          ABH3用CAN サンプルソフト(ドライバ)
*******************************************************************************
@date           2021.02.26
@author         T.Furusawa
@note           ・初版
******************************************************************************/

/******************************************************************************
  インクルードファイル
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <time.h>
#include <sys/time.h>

#include "lib.h"
#include "canABH3.h"

/******************************************************************************
  型定義
******************************************************************************/
/*! mlWork型構造体
*/
typedef struct _mlWork {
  long PGN;
  short Real_Byte_Size;
  short Byte_Size;
  short Packet_Size;
  short Packet_Max_Size;
  short Packet_Limit_size;
  short Packet_Start_Number;
  short Packet_All_Counter;
  short Packet_Local_Counter;
  short Byte_Counter;
  char Status;
  char *Buffer;
} mlWork;

/******************************************************************************
  定数定義
******************************************************************************/

//  min/maxの定数定義
#define min(x, y) (((x)<(y))?(x):(y))
#define max(x, y) (((x)>(y))?(x):(y))

// CAN ID 関連

#define SourceBit       (0)
#define DestinationBit  (8)
#define PDUFormatBit    (16)
#define DataPageBit     (24)
#define PriorityBit     (26)

#define DP0 (0 << DataPageBit)
#define DP1 (1 << DataPageBit)

#define SOURCE(s)       (s << SourceBit)
#define DESTINATION(s)  (s << DestinationBit)
#define PDUFORMAT(p)    (p << PDUFormatBit)
#define PRIORITY(p)     (p << PriorityBit)

#define Mask_Normal     (0x03ffffff)
#define Mask_No_PGN     (0x0300ffff)

// PGN Number

#define ProprietaryAPGN (0xef)
#define BroadRequestPGN (0xea)
#define ProprietaryBPGN (0xff)
#define MultiCtrlPGN    (0xec)
#define MultiDataPGN    (0xeb)

// Multi Packet Control Byte

#define CM_RTS          (0x10)
#define CM_CTS          (0x11)
#define CM_EndOfMsgACK  (0x13)
#define Conn_Abort      (0xff)

// Abort Reason

#define Abort_Double    (0x01)
#define Abort_Lacking   (0x02)
#define Abort_Timeout   (0x03)

// Multi Packet Time OUt

#define MULTI_TIME_OUT  (1250)

/******************************************************************************
  ローカル関数
******************************************************************************/
struct timespec req;

/******************************************************************************
  グローバル関数
******************************************************************************/

/*  速度データの浮動小数点形式からCAN形式への変換
@param[in]      vel         浮動小数点で表した単位[min-1]の速度データ
@return                     CANで使用する、整数で表した単位[0.2min-1]の速度データ　-6553.6[min-1]～0[min-1]～6553.4[min-1]
*/
short cnvVel2CAN(float vel)
{
  long l;

  l = vel / UNIT_VEL;
  l = min(l, (short)0x7fff);
  l = max(l, (short)0x8000);
  
  return (short)l;
}

/*  速度データのCAN形式から浮動小数点形式への変換
@param[in]      vel         CANで使用する、整数で表した単位[0.2min-1]の速度データ　-6553.6[min-1]～0[min-1]～6553.4[min-1]
@return                     浮動小数点で表した単位[min-1]の速度データ
*/
float cnvCAN2Vel(short vel)
{
  return vel * UNIT_VEL;
}

/*  電流データの浮動小数点形式からCAN形式への変換
@param[in]      cur         浮動小数点で表した単位[%]の電流データ
@return                     CANで使用する、整数で表した単位[0.01%]の電流データ　-327.68[%]～0.00[%]～327.67[%]
*/
short cnvCur2CAN(float cur)
{
  long l;

l = cur / UNIT_CUR;
l = min(l, (short)0x7fff);
l = max(l, (short)0x8000);

return (short)l;
}

/*  電流データのCAN形式から浮動小数点形式への変換
@param[in]      cur         CANで使用する、整数で表した単位[0.01%]の電流データ　-327.68[%]～0.00[%]～327.67[%]
@return                     浮動小数点で表した単位[%]の電流データ
*/
float cnvCAN2Cur(short cur)
{
  return cur * UNIT_CUR;
}

/*  負荷率データのCAN形式から浮動小数点形式への変換
@param[in]      load        CANで使用する、整数で表した単位[1%]の負荷率データ　0.0[%]～3276.7[%]
@return                     浮動小数点で表した単位[%]の負荷率データ
*/
float cnvCAN2Load(short load)
{
  return load * UNIT_LOAD;
}

/*  アナログ入力データのCAN形式から浮動小数点形式への変換
@param[in]      analog      CANで使用する、整数で表した単位[0.01V]のアナログ入力データ　-327.68[V]～0.00[V]～327.67[V]
@return                     浮動小数点で表した単位[V]のアナログ入力データ
*/
float cnvCAN2Analog(short analog)
{
  return analog * UNIT_ANALOG;
}

/*  電源電圧データのCAN形式から浮動小数点形式への変換
@param[in]      volt        CANで使用する、整数で表した単位[0.01V]の電源電圧データ　0.0[V]～3276.7[V]
@return                     浮動小数点で表した単位[V]の電源電圧データ
*/
float cnvCAN2Volt(short volt)
{
  return volt * UNIT_VOLT;
}

/*  ショートからCAN形式への変換
@param[in]      ptr         保存するcharバッファへのポインタ
@param[in]      s           ショート(2byte)データ
@return                     なし
*/
void cnvShort2CAN(char * ptr, short s)
{
  unsigned short us;

  us = s;
  ptr[0] = s % 256;
  ptr[1] = s / 256;
}

//-----------------------------------------------------------------------------------------------------
int can_send(CAN_ABH3 *abh3Ptr, long id, char *data, int len)
{
  struct can_frame frame;
 
	memset(&frame, 0, sizeof(frame));
  frame.can_id = id|CAN_EFF_FLAG;
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

	/* send frame */
	if (write(abh3Ptr->can.socket, &frame, 16) != 16) {
		printf("Error: write\n");
		return 1;
	}

  return 0;
}

//-----------------------------------------------------------------------------------------------------

int can_open(CAN_ABH3 *abh3Ptr)
{
  struct ifreq ifr;
  struct sockaddr_can addr;

  printf("open\n");

  abh3Ptr->can.socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (abh3Ptr->can.socket < 0) {
    printf("Error: socket\n");
    return 1;
  }

	strncpy(ifr.ifr_name, abh3Ptr->device, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		printf("Error: if_nametoindex\n");
		return 1;
	}
  
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(abh3Ptr->can.socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    printf("Error: bind\n");
    return 1;
  }

  FD_ZERO(&abh3Ptr->can.rdfs);
  FD_SET(abh3Ptr->can.socket, &abh3Ptr->can.rdfs);

  return 0;
}

int can_close(CAN_ABH3 *abh3Ptr)
{
  int err;

	err = close(abh3Ptr->can.socket);
  if (err) {
    printf("Error: close %d\n", err);
    return err;
  }

  return 0;
}

int can_recv(CAN_ABH3 *abh3Ptr, long id, CAN_ABH3_DATA *canData, long tout)
{
  int i, ret;
  struct can_frame frame;
  struct	timeval timeout;

  while(1) {
    if (tout) {
      timeout.tv_sec = tout / 1000;
      timeout.tv_usec = (tout % 1000) * 1000;
    }
    else {
      timeout.tv_sec = abh3Ptr->timeOut / 1000;
      timeout.tv_usec = (abh3Ptr->timeOut % 1000) * 1000;
    }

    ret = select(abh3Ptr->can.socket+1, &abh3Ptr->can.rdfs, NULL, NULL, &timeout);
    switch(ret) {
    case 0:
      printf("Error: time out\n");
      return 1;
    case -1:
      perror("select");
      return 2;
    }

    ret = read(abh3Ptr->can.socket, &frame, sizeof(frame));
    if ((frame.can_id & 0x03ffffff) == id) {
      memcpy(canData, frame.data, frame.can_dlc);

      break;
    }
  }

  return 0;
}

int can_recv_NO_PGN(CAN_ABH3 *abh3Ptr, long *id, CAN_ABH3_DATA *canData, long tout)
{
  int i, ret;
  struct can_frame frame;
  struct	timeval timeout;

  while(1) {
    if (tout) {
      timeout.tv_sec = tout / 1000;
      timeout.tv_usec = (tout % 1000) * 1000;
    }
    else {
      timeout.tv_sec = abh3Ptr->timeOut / 1000;
      timeout.tv_usec = (abh3Ptr->timeOut % 1000) * 1000;
    }

    ret = select(abh3Ptr->can.socket+1, &abh3Ptr->can.rdfs, NULL, NULL, &timeout);
    switch(ret) {
    case 0:
      printf("Error: time out\n");
      return 1;
    case -1:
      perror("select");
      return 2;
    }

    ret = read(abh3Ptr->can.socket, &frame, sizeof(frame));
    if ((frame.can_id & 0x0300ffff) == *id) {
      memcpy(canData, frame.data, frame.can_dlc);
      *id = frame.can_id & 0x03ffffff;

      break;
    }
  }

  return 0;
}

//-----------------------------------------------------------------------------------------------------

/*  指令の初期化
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_init(CAN_ABH3 *abh3Ptr)
{
  int err;
  CAN_ABH3_DATA canData;

  abh3Ptr->pgn = ProprietaryAPGN;

  // シングルパケット　CAN ID
  abh3Ptr->id.singleDP0Send = PRIORITY(abh3Ptr->priority) + DP0 + PDUFORMAT(abh3Ptr->pgn) + DESTINATION(abh3Ptr->abh3ID) + SOURCE(abh3Ptr->hostID);
  abh3Ptr->id.singleDP0Recv =                               DP0 + PDUFORMAT(abh3Ptr->pgn) + DESTINATION(abh3Ptr->hostID) + SOURCE(abh3Ptr->abh3ID);
  abh3Ptr->id.singleDP1Send = PRIORITY(abh3Ptr->priority) + DP1 + PDUFORMAT(abh3Ptr->pgn) + DESTINATION(abh3Ptr->abh3ID) + SOURCE(abh3Ptr->hostID);
  abh3Ptr->id.singleDP1Recv =                               DP1 + PDUFORMAT(abh3Ptr->pgn) + DESTINATION(abh3Ptr->hostID) + SOURCE(abh3Ptr->abh3ID);

  // ブロードキャスト　CAN ID
  abh3Ptr->id.broadReqSend  = PRIORITY(abh3Ptr->priority) + DP0 + PDUFORMAT(BroadRequestPGN) + DESTINATION(abh3Ptr->abh3ID) + SOURCE(abh3Ptr->hostID);
  abh3Ptr->id.broadBaseRecv =                               DP0 + PDUFORMAT(ProprietaryBPGN)                                + SOURCE(abh3Ptr->abh3ID);

  // マルチキャスト　CAN ID
  abh3Ptr->id.multiRecv     = PRIORITY(abh3Ptr->priority) + DP0                           + DESTINATION(abh3Ptr->hostID) + SOURCE(abh3Ptr->abh3ID);
  abh3Ptr->id.multiCtrlSend = PRIORITY(abh3Ptr->priority) + DP0 + PDUFORMAT(MultiCtrlPGN) + DESTINATION(abh3Ptr->abh3ID) + SOURCE(abh3Ptr->hostID);
  abh3Ptr->id.multiCtrlRecv =                               DP0 + PDUFORMAT(MultiCtrlPGN) + DESTINATION(abh3Ptr->hostID) + SOURCE(abh3Ptr->abh3ID);
  abh3Ptr->id.multiDataSend = PRIORITY(abh3Ptr->priority) + DP0 + PDUFORMAT(MultiDataPGN) + DESTINATION(abh3Ptr->abh3ID) + SOURCE(abh3Ptr->hostID);
  abh3Ptr->id.multiDataRecv =                               DP0 + PDUFORMAT(MultiDataPGN) + DESTINATION(abh3Ptr->hostID) + SOURCE(abh3Ptr->abh3ID);

  // 指令の初期化
  abh3Ptr->singleDP0.cmdAY = abh3Ptr->singleDP0.cmdBX = abh3Ptr->singleDP0.input = 0;
  err = can_open(abh3Ptr);
  if (err) {
    return err;
  }

  // 指令の送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP0Send, (char *)&(abh3Ptr->singleDP0), 8);
  if (err) {
    return err;
  }
  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP0Recv, &canData, 0);

  return err;
}

int abh3_can_finish(CAN_ABH3 *abh3Ptr)
{
  int err;

  err = can_close(abh3Ptr);

  return err;
}

/*  指令の送信（軸別）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      cmd         指令値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_cmdAY(CAN_ABH3 *abh3Ptr, short cmd, CAN_ABH3_DATA *ptr)
{
  int err;

  // 指令の設定
  abh3Ptr->singleDP0.cmdAY = cmd;
  
  // 指令の送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP0Send, (char *)&(abh3Ptr->singleDP0), 8);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP0Recv, ptr, 0);

  return err;
}

int abh3_can_cmdBX(CAN_ABH3 *abh3Ptr, short cmd, CAN_ABH3_DATA *ptr)
{
  int err;

  // 指令の設定
  abh3Ptr->singleDP0.cmdBX = cmd;
  
  // 指令の送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP0Send, (char *)&(abh3Ptr->singleDP0), 8);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP0Recv, ptr, 0);

  return err;
}

/*  指令の送信（同時）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      cmdAY       A/Y指令値
@param[in]      cmdBX       B/X指令値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_cmd(CAN_ABH3 *abh3Ptr, short cmdAY, short cmdBX, CAN_ABH3_DATA *ptr)
{
  int err;

  // 指令の設定
  abh3Ptr->singleDP0.cmdAY = cmdAY;
  abh3Ptr->singleDP0.cmdBX = cmdBX;
  
  // 指令の送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP0Send, (char *)&(abh3Ptr->singleDP0), 8);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP0Recv, ptr, 0);

  return err;
}

/*  入力の送信（一括）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      data        データ値
@param[in]      mask        マスク値
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_inSet(CAN_ABH3 *abh3Ptr, long data, long mask, CAN_ABH3_DATA *ptr)
{
  int err;

  // 指令の設定
  abh3Ptr->singleDP0.input = (abh3Ptr->singleDP0.input & ~mask) | (data & mask);

  // 指令の送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP0Send, (char *)&(abh3Ptr->singleDP0), 8);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP0Recv, ptr, 0);

  return err;
}

/*  入力の送信（ビット）
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      num         ビット番号(0～31)
@param[in]      data        設定データ(0～1)
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_inBitSet(CAN_ABH3 *abh3Ptr, char num, char data, CAN_ABH3_DATA *ptr)
{
  int err;

  // 指令の設定
  abh3Ptr->singleDP0.input = (abh3Ptr->singleDP0.input & ~(1 << num)) | (data << num);

  // 指令の送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP0Send, (char *)&(abh3Ptr->singleDP0), 8);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP0Recv, ptr, 0);

  return err;
}

/*  積算値のリクエスト
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_reqPulse(CAN_ABH3 *abh3Ptr, CAN_ABH3_DATA *ptr)
{
  int err;

  // リクエストの送受信
  err = can_send(abh3Ptr, abh3Ptr->id.singleDP1Send, (char *)NULL, 0);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.singleDP1Recv, ptr, 0);

  return err;
}

/*  ブロードキャストパケットのリクエスト
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      num         番号(0x00～0xff)
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_reqBRD(CAN_ABH3 *abh3Ptr, int num, CAN_ABH3_DATA *ptr)
{
  int err;
  char buf[3] = {0x00, 0xff, 0x00};

  // リクエストの送受信
  buf[0] = num;
  err = can_send(abh3Ptr, abh3Ptr->id.broadReqSend, buf, 3);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.broadBaseRecv + DESTINATION(num), ptr, 0);

  return err;
}

/*  シングルパケットのリクエスト
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      num         番号(0x00～0x01)
@param[in]      ptr         戻り値の構造体へのポインタ
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_reqSNG(CAN_ABH3 *abh3Ptr, int num, CAN_ABH3_DATA *ptr)
{
  int err;
  char buf[3] = {0x00, 0x00, 0x00};

  // リクエストの送受信
  buf[0] = num;
  buf[1] = abh3Ptr->pgn;
  
  err = can_send(abh3Ptr, abh3Ptr->id.broadReqSend, buf, 3);
  if (err) {
    return err;
  }

  err = can_recv(abh3Ptr, abh3Ptr->id.broadBaseRecv + DESTINATION(num), ptr, 0);

  return err;
}

/*  マルチパケットによるTelABH3パケットの送受信
@param[in]      abh3Ptr     CAN_ABH3構造体へのポインタ
@param[in]      sbuf        送信文字列へのポインタ(TelABH3プロトコル受信データフォーマット)
@param[in]      rbuf        受信文字列へのポインタ(TelABH3プロトコル送信データフォーマット)
@return                     エラー状態（0：正常、0以外：異常）
*/
int abh3_can_trans(CAN_ABH3 *abh3Ptr, char *sbuf, char *rbuf)
{
  int i, j, err, flag, num;
  long id;
  mlWork mlw;
  CAN_ABH3_DATA dt, dt1;

  // 初期化
  mlw.Real_Byte_Size = strlen(sbuf);
  mlw.Byte_Size = mlw.Real_Byte_Size < 9 ? 9 : mlw.Real_Byte_Size;
  mlw.Packet_Size = mlw.Byte_Size / 7;
  if (mlw.Byte_Size % 7) {
    mlw.Packet_Size++;
  }
  mlw.Packet_Max_Size = mlw.Packet_Size;
  mlw.Packet_Limit_size = 16; // 固定
  mlw.Packet_Start_Number = 1;
  mlw.Packet_All_Counter = 0;
  mlw.Byte_Counter = 0;

  // RTS送信
  dt.raw.buf[0] = CM_RTS;
  cnvShort2CAN((char *)&dt.raw.buf[1], mlw.Byte_Size);
  dt.raw.buf[3] = mlw.Packet_Size;
  dt.raw.buf[4] = mlw.Packet_Max_Size;
  dt.raw.buf[5] = 0;
  dt.raw.buf[6] = abh3Ptr->pgn;
  dt.raw.buf[7] = 0;
  err = can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
  if (err) {
		printf("Error: RTS Send\n");
    return err;
  }

 dt1.raw.buf[0] = 0;
  for(flag=0; !flag; ){
    // CTS受信
    err = can_recv(abh3Ptr, abh3Ptr->id.multiCtrlRecv, &dt, MULTI_TIME_OUT);
    if (err) {
      if (err==1) {
        dt.raw.buf[0] = Conn_Abort;
        dt.raw.buf[1] = Abort_Timeout;
        dt.raw.buf[2] = 0xff;
        dt.raw.buf[3] = 0xff;
        dt.raw.buf[4] = 0xff;
        dt.raw.buf[5] = 0;
        dt.raw.buf[6] = abh3Ptr->pgn;
        dt.raw.buf[7] = 0;
        can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
      }
  		printf("Error: CTS Recv\n");
      return err;
    }


  // RTS送信
  if ( dt1.raw.buf[0] == 0) {
    dt1.raw.buf[0] = CM_RTS;
    cnvShort2CAN((char *)&dt1.raw.buf[1], mlw.Byte_Size);
    dt1.raw.buf[3] = mlw.Packet_Size;
    dt1.raw.buf[4] = mlw.Packet_Max_Size;
    dt1.raw.buf[5] = 0;
    dt1.raw.buf[6] = abh3Ptr->pgn;
    dt1.raw.buf[7] = 0;
    err = can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt1.raw.buf, 8);
    if (err) {
      printf("Error: RTS Send\n");
      return err;
    }
  }

    switch(dt.raw.buf[0]) {
    case CM_CTS:
      // 送信初期化
      mlw.Packet_Limit_size = dt.raw.buf[1];
      mlw.Packet_Start_Number = dt.raw.buf[2];

      // Data送信
        for (i=mlw.Packet_Start_Number; i<mlw.Packet_Start_Number+mlw.Packet_Limit_size; i++) {
          dt.raw.buf[0] = i;
          for (j=0; j<7; j++) {


            if ((i-1)*7+j >= mlw.Byte_Size) {
              dt.raw.buf[1+j] = 255;  // Padding
            }
            else if ((i-1)*7+j >= mlw.Real_Byte_Size) {
              dt.raw.buf[1+j] = 0x16; // Under 9 character
            }
            else {
              dt.raw.buf[1+j] = sbuf[(i-1)*7+j];
            }
          }
          err = can_send(abh3Ptr, abh3Ptr->id.multiDataSend, dt.raw.buf, 8);
          if (err) {
        		printf("Error: DT Send\n");
            return err;
          }

          mlw.Packet_All_Counter++;
        }
      break;
    case CM_EndOfMsgACK:
      flag = 1;
      break;
    case Conn_Abort:
  		printf("Error: Connection Abort\n");
      break;
    default:
  		printf("Error: Data Send default [%02x]\n", dt.raw.buf[0]);
      for (i=0; i<8; i++) {
        printf("%02x ", dt.raw.buf[i]);
      }
      printf("\n");
      return 3;
    }
  }

  // RTS受信
  err = can_recv(abh3Ptr, abh3Ptr->id.multiCtrlRecv, &dt, MULTI_TIME_OUT);
  if (err) {
    if (err==1) {
      dt.raw.buf[0] = Conn_Abort;
      dt.raw.buf[1] = Abort_Timeout;
      dt.raw.buf[2] = 0xff;
      dt.raw.buf[3] = 0xff;
      dt.raw.buf[4] = 0xff;
      dt.raw.buf[5] = 0;
      dt.raw.buf[6] = abh3Ptr->pgn;
      dt.raw.buf[7] = 0;
      can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
    }
		printf("Error: RTS Recv\n");
    return err;
  }

  if (dt.raw.buf[0] != CM_RTS) {
		printf("Error: RTS Recv not RTS\n");
    return 3;
  }
  else {
    mlw.Byte_Size = ((int)dt.raw.buf[2]<<8) + dt.raw.buf[1];
    mlw.Packet_Size = dt.raw.buf[3];
    mlw.Packet_Max_Size = dt.raw.buf[4];
    mlw.Packet_Limit_size = 16; // 固定
    mlw.Packet_Start_Number = 1;
    mlw.Packet_All_Counter = 0;
    mlw.Packet_Local_Counter = 0;
    mlw.Byte_Counter = 0;
    mlw.Buffer = rbuf;

    // 初回　CTS送信
    dt.raw.buf[0] = CM_CTS;
    i = min(mlw.Packet_Size, mlw.Packet_Max_Size);
    dt.raw.buf[1] = min(i, mlw.Packet_Limit_size);
    dt.raw.buf[2] = mlw.Packet_Start_Number;
    dt.raw.buf[3] = 255;
    dt.raw.buf[4] = 255;
    dt.raw.buf[5] = 0;
    dt.raw.buf[6] = abh3Ptr->pgn;
    dt.raw.buf[7] = 0;
    err = can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
    if (err) {
  		printf("Error: CTS First Send\n");
      return 4;
    }

    // マルチパケットパケット　データ：DT受信
    for(flag=0; !flag;) {
      id = abh3Ptr->id.multiRecv;
      err = can_recv_NO_PGN(abh3Ptr, &id, &dt, MULTI_TIME_OUT);
      if (err) {
        if (err==1) {
          dt.raw.buf[0] = Conn_Abort;
          dt.raw.buf[1] = Abort_Timeout;
          dt.raw.buf[2] = 0xff;
          dt.raw.buf[3] = 0xff;
          dt.raw.buf[4] = 0xff;
          dt.raw.buf[5] = 0;
          dt.raw.buf[6] = abh3Ptr->pgn;
          dt.raw.buf[7] = 0;
          can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
        }
  		  printf("Error: DT Recv\n");
        return 5;
      }

      if (id==abh3Ptr->id.multiCtrlRecv) {
        switch (dt.raw.buf[0]) {
        case Conn_Abort:
          mlw.Buffer[0] = '\0';
          printf("Abort Receive\n");
          flag = 1;
          break;
        case CM_RTS:
          dt.raw.buf[0] = Conn_Abort;
          dt.raw.buf[1] = Abort_Double;
          dt.raw.buf[2] = 0xff;
          dt.raw.buf[3] = 0xff;
          dt.raw.buf[4] = 0xff;
          dt.raw.buf[5] = 0;
          dt.raw.buf[6] = abh3Ptr->pgn;
          dt.raw.buf[7] = 0;
          can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
          printf("Send Double\n");
          break;
        default:
          break;
        }
      }
      else if (id==abh3Ptr->id.multiDataRecv) {
        mlw.Packet_All_Counter++;
        mlw.Packet_Local_Counter++;
        if (mlw.Packet_All_Counter == mlw.Packet_Size) {
          // 最後のパケット
          num = mlw.Byte_Size - mlw.Byte_Counter;

          for (i=0; i<num; i++) {
            mlw.Buffer[mlw.Byte_Counter+i] = dt.raw.buf[i+1];
          }
          mlw.Byte_Counter += num;
          mlw.Buffer[mlw.Byte_Counter] = '\0';

          dt.raw.buf[0] = CM_EndOfMsgACK;
          dt.raw.buf[1] = mlw.Byte_Counter & 0xff;
          dt.raw.buf[2] = (mlw.Byte_Counter >>  8) & 0xff;
          dt.raw.buf[3] = mlw.Packet_All_Counter;
          dt.raw.buf[4] = 255;
          dt.raw.buf[5] = 0;
          dt.raw.buf[6] = abh3Ptr->pgn;
          dt.raw.buf[7] = 0;

          err = can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
          if (err) {
            printf("Error: CTS Cont Send\n");
            return 6;
          }
          flag=1;
        }
        else {
          // 継続パケット
          num = 7;

          for (i=0; i<num; i++) {
            mlw.Buffer[mlw.Byte_Counter+i] = dt.raw.buf[i+1];
          }
          mlw.Byte_Counter += num;
          mlw.Buffer[mlw.Byte_Counter] = '\0';

          // 指定パケット数受信　CTS送信
          if (mlw.Packet_Local_Counter >= mlw.Packet_Limit_size) {
            mlw.Packet_Start_Number = mlw.Packet_All_Counter + 1;
            dt.raw.buf[0] = CM_CTS;
            i = min(mlw.Packet_Size-mlw.Packet_All_Counter, mlw.Packet_Max_Size);
            dt.raw.buf[1] = min(i, mlw.Packet_Limit_size);
            dt.raw.buf[2] = mlw.Packet_Start_Number;
            dt.raw.buf[3] = 255;
            dt.raw.buf[4] = 255;
            dt.raw.buf[5] = 0;
            dt.raw.buf[6] = abh3Ptr->pgn;
            dt.raw.buf[7] = 0;

            err = can_send(abh3Ptr, abh3Ptr->id.multiCtrlSend, dt.raw.buf, 8);
            if (err) {
              printf("Error: CTS Last Send\n");
              return 7;
            }

            mlw.Packet_Local_Counter = 0;
          }
        }
      }
      else {
        printf("Error: Iligal PGN\n");
        return 6;
      }
    }
  }

  return err;
}

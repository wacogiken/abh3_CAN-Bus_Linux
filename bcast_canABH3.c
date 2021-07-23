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
@file           bcast_canABH3.c
*******************************************************************************
@brief          ABH3用CAN サンプルソフト(アプリケーション)
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
#include <math.h>

#include "canABH3.h"

#include <time.h>
#include <sys/time.h>

struct timespec req1;

/******************************************************************************
  アプリケーション・メイン
******************************************************************************/
CAN_ABH3 canABH3;
CAN_ABH3_DATA canData;

int broadCastLoop(int flag, int cnt, int ms)
{
  int err, i, j;

  for(i=0; i<cnt; i++) {
    for(j=0; j<7; j++) {
      if (flag) {
        // 送信ID = ブロードキャスト
        err = abh3_can_reqBRDBRD(&canABH3, canABH3.broadGroup*8+j, &canData);
      }
      else {
        // 送信ID = ホストID
        err = abh3_can_reqBRD(&canABH3, canABH3.broadGroup*8+j, &canData);
      }
      if (err) {
        printf("error: %d\n", err);
        return err;
      }
      switch(j) {
      case 0:
        printf("Error: %08x  Alarm: %08x\n", \
          canData.broad0.error, \
          canData.broad0.alarm \
        );
        break;
      case 1:
        printf("control: %08x  in_out: %08x\n", \
          canData.broad1.control, \
          canData.broad1.in_out \
        );
        break; 
      case 2:
        printf("velCmdAY: %10.6f  velCmdBX: %10.6f  velFbkAY: %10.6f  velFbkBX: %10.6f\n", \
          cnvCAN2Vel(canData.broad2.velCmdAY), \
          cnvCAN2Vel(canData.broad2.velCmdBX), \
          cnvCAN2Vel(canData.broad2.velFbkAY), \
          cnvCAN2Vel(canData.broad2.velFbkBX)
        );
        break;
      case 3:
        printf("curCmdAY: %10.6f  curCmdBX: %10.6f  loadA: %10.6f  loadB: %10.6f\n", \
          cnvCAN2Cur(canData.broad3.curCmdAY), \
          cnvCAN2Cur(canData.broad3.curCmdBX), \
          cnvCAN2Load(canData.broad3.loadA), \
          cnvCAN2Load(canData.broad3.loadB)
        );
        break;
      case 4:
        printf("pulseA: %d  pulseB: %d\n", \
          canData.broad4.pulseA, \
          canData.broad4.pulseB \
        );
        break; 
      case 5:
        printf("analog0: %10.6f  analog1: %10.6f  mainVolt: %10.6f  controlVolt: %10.6f\n", \
          cnvCAN2Analog(canData.broad5.analog0), \
          cnvCAN2Analog(canData.broad5.analog1), \
          cnvCAN2Volt(canData.broad5.mainVolt), \
          cnvCAN2Volt(canData.broad5.controlVolt)
        );
        break;
      case 6:
        printf("monitor0: %10.6f  monitor1: %10.6f\n", \
          canData.broad6.monitor0, \
          canData.broad6.monitor1 \
        );
        break; 
      case 7:
        printf("BRD7: %d %d %d %d %d %d %d %d\n", \
          canData.raw.buf[0], \
          canData.raw.buf[1], \
          canData.raw.buf[2], \
          canData.raw.buf[3], \
          canData.raw.buf[4], \
          canData.raw.buf[5], \
          canData.raw.buf[6], \
          canData.raw.buf[7] \
        );
        break; 
      }

      usleep(ms*1000);
    }
    printf("\n");
  }

  return 0;
}

int main(int argc, char *argv[])
{
  int err;
  char *sbuf, rbuf[BUFSIZ];
  float f;

  // 初期化
  err = abh3_can_port_init(&canABH3, "can0", 1, 2, 0, 5, 1000);
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  req1.tv_sec  = 0;
  req1.tv_nsec = 10000000; // 50ms

  printf("Single Pcket DP0 Send ID: %08lx\n", canABH3.id.singleDP0Send);
  printf("                 Recv ID: %08lx\n", canABH3.id.singleDP0Recv);
  printf("Broad Packet Send Resuest ID: %08lx\n", canABH3.id.broadReqSend);
  printf("             Recv    Base ID: %08lx\n\n", canABH3.id.broadBaseRecv);

  err = broadCastLoop(0, 1, 10);
  if (err) {
    return err;
  }

  err = broadCastLoop(1, 1, 10);
  if (err) {
    return err;
  }

  err = abh3_can_finish(&canABH3);
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  return 0;
}

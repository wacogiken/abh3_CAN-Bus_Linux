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
@file           test_canABH3.c
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
#include "pack_float.h"

#include <time.h>
#include <sys/time.h>

struct timespec req1;

/******************************************************************************
  アプリケーション・メイン
******************************************************************************/
CAN_ABH3 canABH3;
CAN_ABH3_DATA canData;

int main(int argc, char *argv[])
{
  int i, j, ret;
  char *sbuf, rbuf[BUFSIZ];
  float f;

  // 初期化
  canABH3.abh3ID = 1;
  canABH3.hostID = 2;
  canABH3.timeOut = 1000;
  canABH3.device = "can0";
  canABH3.priority = 0;
  canABH3.broadGroup = 5;

  req1.tv_sec  = 0;
  req1.tv_nsec = 10000000; // 10ms

  // 接続
  ret = abh3_can_init(&canABH3);
  if (ret) {
    printf("error: %d\n", ret);
    return(0);
  }

  printf("Single Pcket DP0 Send ID: %08lx\n", canABH3.id.singleDP0Send);
  printf("                 Recv ID: %08lx\n", canABH3.id.singleDP0Recv);
  printf("Single Pcket DP1 Send ID: %08lx\n", canABH3.id.singleDP1Send);
  printf("                 Recv ID: %08lx\n", canABH3.id.singleDP1Recv);
  printf("Broad Packet Send Resuest ID: %08lx\n", canABH3.id.broadReqSend);
  printf("             Recv    Base ID: %08lx\n\n", canABH3.id.broadBaseRecv);

  // エラー解除
  sbuf = "CP7001";
  abh3_can_inSet(&canABH3, 0x80000000, 0xffffffff, &canData);
  while(1) {
    abh3_can_reqBRD(&canABH3, canABH3.broadGroup*8+0, &canData);
    printf("Abnormal: %08x  Warning: %08x\n", \
      canData.broad0.abnormal, \
      canData.broad0.warning \
    );
    if (canData.broad0.abnormal) {
      abh3_can_inSet(&canABH3, 0x80000000, 0xffffffff, &canData);
    }
    else {
      break;
    }
  }
  abh3_can_inSet(&canABH3, 0x00000000, 0xffffffff, &canData);

  // サーボON
  abh3_can_inSet(&canABH3, 0x00003003, 0xffffffff, &canData);
  for(i=0; i<1000; i++) {
    // 指令
    ret = abh3_can_cmd(&canABH3, cnvVel2CAN(100), cnvVel2CAN(-50), &canData);
    printf("%10.3f %10.3f %10.3f %10.3f\n",
      cnvCAN2Vel(canData.singleDP0.fbk.Y), 
      cnvCAN2Vel(canData.singleDP0.fbk.X),
      cnvCAN2Vel(canData.singleDP0.fbk.A),
      cnvCAN2Vel(canData.singleDP0.fbk.B)
    );

    nanosleep(&req1, NULL);
  }
  // サーボOFF
  abh3_can_inSet(&canABH3, 0x00000000, 0xffffffff, &canData);

  // 切断
  ret = abh3_can_finish(&canABH3);
  if (ret) {
    printf("error: %d\n", ret);
    return(ret);
  }

  return 0;
}

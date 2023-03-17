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
@file           single_canABH3.c
*******************************************************************************
@brief          ABH3用CAN サンプルソフト(アプリケーション)
*******************************************************************************
@date           2021.02.26
@author         T.Furusawa
@note           ・初版

@date           2023.01.31
@author         T.Furusawa
@note           ・CAN新仕様に対応
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
#include <string.h>

  struct timespec req1;

/******************************************************************************
  アプリケーション・メイン
******************************************************************************/
CAN_ABH3 canABH3;
CAN_ABH3_DATA canData;

int main(int argc, char *argv[])
{
  int i, j, err=1;
  float f;

  // 初期化
  req1.tv_sec  = 0;
  req1.tv_nsec = 10000000; // 100ms

  // 接続
  while (err != 0) {
    err = abh3_can_port_init(&canABH3, "can0", 1, 2, 0, 5, 1000);
    if (err) {
      printf("port init error: %d\n", err);
      abh3_can_finish(&canABH3);
    }
    else {
      err = abh3_can_cmd_init(&canABH3);
      if (err) {
        printf("cmd init error: %d\n", err);
        abh3_can_finish(&canABH3);
      }
    }
  }

  printf("Single Pcket DP0 Send ID: %08lx\n", canABH3.id.singleDP0Send);
  printf("                 Recv ID: %08lx\n", canABH3.id.singleDP0Recv);
  printf("Broad Packet Send Resuest ID: %08lx\n", canABH3.id.broadReqSend);
  printf("             Recv    Base ID: %08lx\n\n", canABH3.id.broadBaseRecv);


  // エラー解除
  abh3_can_inSet(&canABH3, 0x00400000, 0xffffffff, &canData);
  while(1) {
    abh3_can_reqBRD(&canABH3, canABH3.broadGroup*8+0, &canData);
    printf("Error: %08x  Alarm: %08x\n", \
      canData.broad0.error, \
      canData.broad0.alarm \
    );
    if (canData.broad0.error) {
      abh3_can_inSet(&canABH3, 0x00400000, 0xffffffff, &canData);
    }
    else {
      break;
    }
  }
  abh3_can_inSet(&canABH3, 0x00000000, 0xffffffff, &canData);

  // サーボON
  abh3_can_inSet(&canABH3, 0x00007373, 0xffffffff, &canData);
  for(i=0; i<200; i++) {
    // 指令
    err = abh3_can_cmd(&canABH3, cnvVel2CAN(100), cnvVel2CAN(-100), &canData);
    printf("%10.3f %10.3f   %08X\n",
      cnvCAN2Vel(canData.singleDP0.fbkAY), 
      cnvCAN2Vel(canData.singleDP0.fbkBX),
      canData.singleDP0.control
    );

    nanosleep(&req1, NULL);
  }
  // サーボOFF
  abh3_can_inSet(&canABH3, 0x00000000, 0xffffffff, &canData);

  // 切断
  err = abh3_can_finish(&canABH3);
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  return 0;
}

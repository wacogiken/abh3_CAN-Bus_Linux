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

@date           2023.02.01
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

#include "canABH3++.hpp"

#include <time.h>
#include <sys/time.h>

struct timespec req1;

/******************************************************************************
  アプリケーション・メイン
******************************************************************************/

#define broadGroup 5
canABH3 abh3;

int broadCastLoop(int flag, int cnt, int ms)
{
  int err, i, j;

  for(i=0; i<cnt; i++) {
    for(j=0; j<7; j++) {
      if (flag) {
        // 送信ID = ブロードキャスト
        err = abh3.reqBRDBRD(broadGroup*8+j);
      }
      else {
        // 送信ID = ホストID
        err = abh3.reqBRD(broadGroup*8+j);
      }
      if (err) {
        printf("error: %d\n", err);
        return err;
      }
      switch(j) {
      case 0:
        printf("Error: %08x  Alarm: %08x\n", \
          abh3.getBroad0Error(), \
          abh3.getBroad0Alarm() \
        );
        break;
      case 1:
        printf("in_out: %08x  input: %08x\n", \
          abh3.getBroad1In_Out(), \
          abh3.getBroad1Input() \
        );
        break; 
      case 2:
        printf("velCmdAY: %10.6f  velCmdBX: %10.6f  velFbkAY: %10.6f  velFbkBX: %10.6f\n", \
          cnvCAN2Vel(abh3.getBroad2VelCmdAY()), \
          cnvCAN2Vel(abh3.getBroad2VelCmdBX()), \
          cnvCAN2Vel(abh3.getBroad2VelFbkAY()), \
          cnvCAN2Vel(abh3.getBroad2VelFbkBX())
        );
        break;
      case 3:
        printf("curCmdAY: %10.6f  curCmdBX: %10.6f  loadA: %10.6f  loadB: %10.6f\n", \
          cnvCAN2Cur(abh3.getBroad3CurCmdAY()), \
          cnvCAN2Cur(abh3.getBroad3CurCmdBX()), \
          cnvCAN2Load(abh3.getBroad3LoadA()), \
          cnvCAN2Load(abh3.getBroad3LoadB())
        );
        break;
      case 4:
        printf("pulseA: %d  pulseB: %d\n", \
          abh3.getBroad4pulseA(), \
          abh3.getBroad4pulseB() \
        );
        break; 
      case 5:
        printf("analog0: %10.6f  analog1: %10.6f  mainVolt: %10.6f  controlVolt: %10.6f\n", \
          cnvCAN2Analog(abh3.getBroad5Analog0()), \
          cnvCAN2Analog(abh3.getBroad5Analog1()), \
          cnvCAN2Volt(abh3.getBroad5MainVolt()), \
          cnvCAN2Volt(abh3.getBroad5ControlVolt())
        );
        break;
      case 6:
        printf("monitor0: %10.6f  monitor1: %10.6f\n", \
          abh3.getBroad6Monitor0(), \
          abh3.getBroad6Monitor1() \
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
  err = abh3.port_init((char *)"can0", 1, 2, 0, broadGroup, 1000);
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  req1.tv_sec  = 0;
  req1.tv_nsec = 10000000; // 50ms

  err = broadCastLoop(0, 1, 10);
  if (err) {
    return err;
  }

  err = broadCastLoop(1, 1, 10);
  if (err) {
    return err;
  }

  err = abh3.finish();
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  return 0;
}

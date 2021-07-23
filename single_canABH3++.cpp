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

#include "canABH3++.hpp"

#include <time.h>
#include <sys/time.h>

struct timespec req1;

/******************************************************************************
  アプリケーション・メイン
******************************************************************************/
// CAN_ABH3 canABH3;
// CAN_ABH3_DATA canData;

int main(int argc, char *argv[])
{
  int i, j, err;
  float f;

  canABH3 abh3;

  // 初期化
  req1.tv_sec  = 0;
  req1.tv_nsec = 10000000; // 10ms

  // 接続
  err = abh3.port_init((char *)"can0", 1, 2, 0, 5, 1000);
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  // エラー解除
  abh3.inSet(0x80000000, 0xffffffff);
  while(1) {
    abh3.reqBRD(5*8+0);
    printf("Error: %08x  Alarm: %08x\n", \
      abh3.getBroad0Error(), \
      abh3.getBroad0Alarm() \
    );
    if (abh3.getBroad0Error()) {
      abh3.inSet(0x80000000, 0xffffffff);
    }
    else {
      break;
    }
  }
  abh3.inSet(0x00000000, 0xffffffff);

  // サーボON
  abh3.inSet(0x00003003, 0xffffffff);
  for(i=0; i<100; i++) {
    // 指令
    err = abh3.cmd(cnvVel2CAN(100), cnvVel2CAN(-50));
    printf("%10.3f %10.3f %10.3f %10.3f\n",
      cnvCAN2Vel(abh3.getSingleDP0FbkY()), 
      cnvCAN2Vel(abh3.getSingleDP0FbkX()),
      cnvCAN2Vel(abh3.getSingleDP0FbkA()),
      cnvCAN2Vel(abh3.getSingleDP0FbkB())
    );

    nanosleep(&req1, NULL);
  }
  // サーボOFF
  abh3.inSet(0x00000000, 0xffffffff);

  // 切断
  err = abh3.finish();
  if (err) {
    printf("error: %d\n", err);
    return err;
  }

  return 0;
}

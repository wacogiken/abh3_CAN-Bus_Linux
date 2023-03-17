extern "C" {
  #include "canABH3.h"
}

// クラス定義
class canABH3 {
protected :
  CAN_ABH3 st;
  CAN_ABH3_DATA dt;
public :
  int port_init(char *device, int abh3ID, int hostID, int priority, int broadGroup, int timeOut);
  int cmd_init();
  int finish();

  int cmdAY(short cmd);
  int cmdBX(short cmd);
  int cmd(short cmdAY, short cmdBX);
  int inSet(long data, long mask);
  int inBitSet(char num, char data);
  int cmdAll(short cmdAY, short cmdBX, long data, long mask);
  int reqBRD(int num);
  int reqBRDBRD(int num);
  int reqSNG(int num);

  int16_t getSingleDP0FbkAY();
  int16_t getSingleDP0FbkBX();
  int32_t getSingleDP0Control();

  int32_t getBroad0Error();
  int32_t getBroad0Alarm();

  int32_t getBroad1In_Out();
  int32_t getBroad1Input();

  int16_t getBroad2VelCmdAY();
  int16_t getBroad2VelCmdBX();
  int16_t getBroad2VelFbkAY();
  int16_t getBroad2VelFbkBX();

  int16_t getBroad3CurCmdAY();
  int16_t getBroad3CurCmdBX();
  int16_t getBroad3LoadA();
  int16_t getBroad3LoadB();

  int32_t getBroad4pulseA();
  int32_t getBroad4pulseB();

  int16_t getBroad5Analog0();
  int16_t getBroad5Analog1();
  int16_t getBroad5MainVolt();
  int16_t getBroad5ControlVolt();

  float getBroad6Monitor0();
  float getBroad6Monitor1();
};

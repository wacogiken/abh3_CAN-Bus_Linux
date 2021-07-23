extern "C" {
  #include "canABH3.h"
}

#include "canABH3++.hpp"

int canABH3::port_init(
  char *device,
  int abh3ID,
  int hostID,
  int priority,
  int broadGroup,
  int timeOut
  )
{
  int err = abh3_can_port_init(&this->st, device, abh3ID, hostID, priority, broadGroup, timeOut);
  
  return err;
}

int canABH3::cmd_init()
{
  int err = abh3_can_cmd_init(&this->st);

  return err;
}

int canABH3::finish()
{
  int err = abh3_can_finish(&this->st);

  return err;
}

int canABH3::cmdAY(short cmd)
{
  int err = abh3_can_cmdAY(&this->st, cmd, &this->dt);

  return err;
}

int canABH3::cmdBX(short cmd)
{
  int err = abh3_can_cmdBX(&this->st, cmd, &this->dt);

  return err;
}

int canABH3::cmd(short cmdAY, short cmdBX)
{
  int err = abh3_can_cmd(&this->st, cmdAY, cmdBX, &this->dt);

  return err;
}

int canABH3::inSet(long data, long mask)
{
  int err = abh3_can_inSet(&this->st, data, mask, &this->dt);

  return err;
}

int canABH3::inBitSet(char num, char data)
{
  int err = abh3_can_inBitSet(&this->st, num, data, &this->dt);

  return err;
}

int canABH3::cmdAll(short cmdAY, short cmdBX,  long data, long mask)
{
  int err = abh3_can_cmdAll(&this->st, cmdAY, cmdBX, data, mask, &this->dt);

  return err;
}

int canABH3::reqBRD(int num)
{
  int err = abh3_can_reqBRD(&this->st, num, &this->dt);

  return err;
}

int canABH3::reqBRDBRD(int num)
{
  int err = abh3_can_reqBRDBRD(&this->st, num, &this->dt);

  return err;
}

int canABH3::reqSNG(int num)
{
  int err = abh3_can_reqSNG(&this->st, num, &this->dt);

  return err;
}

int16_t canABH3::getSingleDP0FbkY()
{
  return this->dt.singleDP0.fbk.Y;
}

int16_t canABH3::getSingleDP0FbkX()
{
  return this->dt.singleDP0.fbk.X;
}

int16_t canABH3::getSingleDP0FbkA()
{
  return this->dt.singleDP0.fbk.A;
}

int16_t canABH3::getSingleDP0FbkB()
{
  return this->dt.singleDP0.fbk.B;
}

int32_t canABH3::getBroad0Error()
{
  return this->dt.broad0.error;
}

int32_t canABH3::getBroad0Alarm()
{
  return this->dt.broad0.alarm;
}

int32_t canABH3::getBroad1Control()
{
  return this->dt.broad1.control;
}

int32_t canABH3::getBroad1In_Out()
{
  return this->dt.broad1.in_out;
}

int16_t canABH3::getBroad2VelCmdAY()
{
  return this->dt.broad2.velCmdAY;
}

int16_t canABH3::getBroad2VelCmdBX()
{
  return this->dt.broad2.velCmdBX;
}

int16_t canABH3::getBroad2VelFbkAY()
{
  return this->dt.broad2.velFbkAY;
}

int16_t canABH3::getBroad2VelFbkBX()
{
  return this->dt.broad2.velFbkBX;
}

int16_t canABH3::getBroad3CurCmdAY()
{
  return this->dt.broad3.curCmdAY;
}

int16_t canABH3::getBroad3CurCmdBX()
{
  return this->dt.broad3.curCmdBX;
}

int16_t canABH3::getBroad3LoadA()
{
  return this->dt.broad3.loadA;
}

int16_t canABH3::getBroad3LoadB()
{
  return this->dt.broad3.loadB;
}

int32_t canABH3::getBroad4pulseA()
{
  return this->dt.broad4.pulseA;
}

int32_t canABH3::getBroad4pulseB()
{
  return this->dt.broad4.pulseB;
}

int16_t canABH3::getBroad5Analog0()
{
  return this->dt.broad5.analog0;
}

int16_t canABH3::getBroad5Analog1()
{
  return this->dt.broad5.analog1;
}

int16_t canABH3::getBroad5MainVolt()
{
  return this->dt.broad5.controlVolt;
}

int16_t canABH3::getBroad5ControlVolt()
{
  return this->dt.broad5.mainVolt;
}

float canABH3::getBroad6Monitor0()
{
  return this->dt.broad6.monitor0;
}

float canABH3::getBroad6Monitor1()
{
  return this->dt.broad6.monitor1;
}

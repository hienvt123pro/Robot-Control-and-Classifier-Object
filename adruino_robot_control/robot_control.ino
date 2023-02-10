#ifndef main_ino
#define main_ino

#include"robot_3dof.h"

#define SpeedDefault 20000
#define Dev 1
#define BitStart 0
#define BitStop 1
#define CalibCmd 2
#define MoveCmd 3
#define AdjustSpeedCmd 0


RobotControl robot(SpeedDefault, SpeedDefault, SpeedDefault, SpeedDefault, SpeedDefault, SpeedDefault);

float code[11], cmd[2], data[6];

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()){
    // read data transfer from pc
    for(int i=1; i<=11; i++){
      String ch = Serial.readStringUntil(',');
      float float_ch = ch.toFloat();
      code[i-1] = float_ch;
    }

    // split data transfer to cmd and data array
    cmd[0] = code[2];
    cmd[1] = code[3];
    data[0] = code[4];
    data[1] = code[5];
    data[2] = code[6];
    data[3] = code[7];
    data[4] = code[8];
    data[5] = code[9];

    // check code is correct
    if (code[0] == BitStart && code[10] == BitStop && code[1] == Dev){
      switch ((int)cmd[0]){
        case CalibCmd:
          if (cmd[1]==1){
            robot.calibJ1();
          }
          else if (cmd[1]==2){
            robot.calibJ2();
          }
          else if (cmd[1]==3){
            robot.calibJ3();
          }
          else if (cmd[1]==4){
            robot.calibHome();
          }
          break;
        case MoveCmd:
          robot.robotGoto(data[0], data[1], data[2]);
          break;
        case AdjustSpeedCmd:
          robot.configSpeed(int(data[0]), int(data[1]));
          break;

      }
    }
  }
}

#endif
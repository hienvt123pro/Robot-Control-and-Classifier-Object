#ifndef main_ino
#define main_ino

#include"robot_3dof.h"

#define SpeedDefault 20000
#define Dev 1.0
#define BitStart 0.0
#define BitStop 1.0
#define CalibCmd 2
#define MoveCmd 3
#define AdjustSpeedCmd 0
#define CheckStateCmd 1
#define ProcessCmd 4
#define SetP00Cmd 5
#define ControlEFCmd 6

RobotControl robot(SpeedDefault, SpeedDefault, SpeedDefault, SpeedDefault, SpeedDefault, SpeedDefault);

float cmd[2], data[6];

byte notAvailable[] = { 0, 1, 1, 0, 1 };
byte isAvailable[] = { 0, 1, 1, 1, 1 };

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() >= 11){
    // read data transfer from pc
    float* code = (float *) malloc(11 * sizeof(float));

    for(int i=0; i<11; i++){
      String ch = Serial.readStringUntil(',');
      float float_ch = ch.toFloat();
      code[i] = float_ch;
    }

    // check code is correct
    if ((code[0] == BitStart) && (code[10] == BitStop) && (code[1] == Dev)){
      //split data transfer to cmd and data array
      cmd[0] = code[2];
      cmd[1] = code[3];
      data[0] = code[4];
      data[1] = code[5];
      data[2] = code[6];
      data[3] = code[7];
      data[4] = code[8];
      data[5] = code[9];

      free(code);

      switch (int(cmd[0])){
        case ControlEFCmd:
          if (int(data[0])==0) {
            robot.airMotorControl(0);
            robot.valveSolenoid(1);
          }
          else {
            robot.valveSolenoid(0);
            robot.airMotorControl(1);
          }
          break;

        case ProcessCmd:
          if (~(data[0]==0 && data[1]==0 && data[2]==0)) {
            robot.robotAutoProcess(data[0], data[1], data[2], data[3], data[4], data[5]);
            Serial.write(isAvailable, sizeof(isAvailable));
          }
          else {
            // the act of reject  
            int time_delay = int(data[3]/data[4]) * 1000;
            delay(time_delay);
            Serial.write(isAvailable, sizeof(isAvailable));
          }
          break;

        case SetP00Cmd:
          robot.intermediate_point[0] = data[0];
          robot.intermediate_point[1] = data[1];
          robot.intermediate_point[2] = data[2];
          break;

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

        case CheckStateCmd:
          bool state = robot.isHomePosition();
          if (state == false) {
            Serial.write(notAvailable, sizeof(notAvailable));
          }
          else {
            Serial.write(isAvailable, sizeof(isAvailable));
          }
          break;
      }
    }

  }
}

#endif
#ifndef robot_3dof_h
#define robot_3dof_h

#include"MultiStepper.h"
#include"AccelStepper.h"

AccelStepper stepperX(1,2,5);
AccelStepper stepperY(1,3,6);
AccelStepper stepperZ(1,4,7);

class RobotControl{
  private:
  const float pi = 3.14159;
  float gearRatio1 = 4.5;
  float gearRatio2 = 4.5;
  float gearRatio3 = 4.5; 
  float ref_theta1 = -97.5;
  float ref_theta2 = 132;
  float ref_theta3 = -109;
  float theta1 = 0, theta2 = 0, theta3 = 0;
  int en = 8;
  int dirX = 5;
  int stepX = 2;
  int endX = 9;
  int dirY = 6;
  int stepY = 3;
  int endY = 10;
  int dirZ = 7;
  int stepZ = 4;
  int endZ = 11;

  int relayAirMotor = 12;
  int solenoid = 13;

  float initX = 0;
  float initY = 100;
  float initZ = -130;

  public:
  float intermediate_point[3];

  /// Constructor
  RobotControl(int vx, int ax, int vy, int ay, int vz, int az) {

    // setup pins for driver
    pinMode(en, OUTPUT);
    digitalWrite(en, LOW);

    pinMode(stepX, OUTPUT);
    pinMode(dirX, OUTPUT);

    pinMode(stepY, OUTPUT);
    pinMode(dirY, OUTPUT);

    pinMode(stepZ, OUTPUT);
    pinMode(dirZ, OUTPUT);

    // setup speed motor
    stepperX.setMaxSpeed(vx);
    stepperX.setSpeed(vx);
    stepperX.setAcceleration(ax);

    stepperY.setMaxSpeed(vy);
    stepperY.setSpeed(vy);
    stepperY.setAcceleration(ay);
      
    stepperZ.setMaxSpeed(vz);
    stepperZ.setSpeed(vz);
    stepperZ.setAcceleration(az);

    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    stepperZ.setCurrentPosition(0);

    // setup pins for limit sensor
    pinMode(endX, INPUT_PULLUP);
    pinMode(endY, INPUT_PULLUP);
    pinMode(endZ, INPUT_PULLUP);

    // air motor
    pinMode(relayAirMotor, OUTPUT);
    pinMode(solenoid, OUTPUT);

    // P00
    intermediate_point[0] = 80;
    intermediate_point[1] = 45;
    intermediate_point[2] = -100;
  }

  /// Calibrate position for the robot, doing this before running.
  void calibJ1() {
    digitalWrite(dirX, HIGH);
    while(digitalRead(endX)) // 500hz
    {
      digitalWrite(stepX, HIGH);
      delayMicroseconds(1000);  
      digitalWrite(stepX, LOW);
      delayMicroseconds(1000);  
    }
    stepperX.setCurrentPosition(0); 
  }

  void calibJ2() {
    digitalWrite(dirY, HIGH);
    while(digitalRead(endY)) // 500hz
    {
      digitalWrite(stepY, HIGH);
      delayMicroseconds(1000);  
      digitalWrite(stepY, LOW);
      delayMicroseconds(1000);  
    }
    stepperY.setCurrentPosition(0); 
  }

  void calibJ3() {
    digitalWrite(dirZ, LOW);
    while(digitalRead(endZ)) // 500hz
    {
      digitalWrite(stepZ, HIGH);
      delayMicroseconds(1000);  
      digitalWrite(stepZ, LOW);
      delayMicroseconds(1000);  
    }
    stepperZ.setCurrentPosition(0); 
  }

  void calibHome() {
    calibJ1();
    calibJ3();
    calibJ2();
    robotGoto(initX, initY, initZ);
  }

  /// Control the robot going to desired position 
  void robotGoto(float theta1, float theta2, float theta3) {
      int pulse_1 = gearRatio1*(theta1-ref_theta1)*200*16/360;
      int pulse_2 = gearRatio2*(theta2-ref_theta2)*200*16/360;
      int pulse_3 = gearRatio3*(theta3-ref_theta3)*200*16/360; 
      stepperX.moveTo(-pulse_1);
      stepperY.moveTo(pulse_2);
      stepperZ.moveTo(-(pulse_3 + pulse_2));
      while((stepperX.distanceToGo()!=0)||(stepperY.distanceToGo()!=0)||(stepperZ.distanceToGo()!=0))
      {
        stepperX.run();
        stepperY.run();
        stepperZ.run();
      }
  }

  /// Setup velocity, accelerate for the robot
  void configSpeed(int v, int a) {
    stepperX.setSpeed(v);
    stepperX.setAcceleration(a);

    stepperY.setSpeed(v);
    stepperY.setAcceleration(a);
      
    stepperZ.setSpeed(v);
    stepperZ.setAcceleration(a);
  }

  /// The robot performs the process of picking and dropping objects
  void robotAutoProcess(float pick_theta1, float pick_theta2, float pick_theta3, float drop_theta1, float drop_theta2, float drop_theta3) {
    // on air motor
    airMotorControl(1);

    // goto pick place
    robotGoto(pick_theta1, pick_theta2, pick_theta3);
    delay(300);

    // goto P00
    robotGoto(intermediate_point[0], intermediate_point[1], intermediate_point[2]);

    // goto drop place
    robotGoto(drop_theta1, drop_theta2, drop_theta3);

    // off air motor and on valve solonoid
    airMotorControl(0);
    valveSolenoid(1);
    delay(600);

    // goto home
    robotGoto(initX, initY, initZ);

    // off valve solenoid
    valveSolenoid(0);

  }

  /// Read the position of robot
  bool isHomePosition() {
    // -3900, -1280, 2120: init steps of 3 step motor X, Y, Z
    if ((stepperX.currentPosition()!=-3900)||(stepperY.currentPosition()!=-1280)||(stepperZ.currentPosition()!=2120)) {
      return false;
    }
    else {
      return true;
    }
  }

  void airMotorControl(int button) {
    if (button == 1) {
      digitalWrite(relayAirMotor, HIGH);
    }
    else {
      digitalWrite(relayAirMotor, LOW);
    }
  }

  void valveSolenoid(int button) {
    if (button == 1) {
      digitalWrite(solenoid, HIGH);
    }
    else {
      digitalWrite(solenoid, LOW);
    }
  }

};

#endif
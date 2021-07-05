#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <EEPROM.h>
#include "Joystick.h"

// Okay I need some way of keeping track of this

/*
 * So the idea is that we have an absolute position coming from the pwm 
 * from the REV robotics encoder and we'll use that to reprogram the 
 * direction the motors turn to calibrate...
 * 
 * If that works then it should be able to center itself from either
 * direction.
 * 
 * So I need
 * 
 * x max < 1024
 * x min >= 0
 * x center = ((xMax - xMin) / 2) + xMin
 * 
 * y max < 1024
 * y min >= 0
 * y center = ((yMax - yMin) / 2) + yMin 
 * 
 * But there's a domain mapping problem too
 * 
 * The encoder centers on the index location, which jumps from 
 * 
 * 3,2,1,0 <-> 1024, 1023, 1022 etc
 * 
 * 
 * Calibrate ?
 * 
 * 1) Move joystick in all directions
 * 2) Write to eeprom
 * 3) Request calibration from controller
 * 
 */

enum JOYSTICK_STATE {
  STATE_BOOT,
  STATE_IDLE,
  STATE_CALIBRATE,
  STATE_ODRIVE_INIT,
  STATE_DCS,
  STATE_GENERIC  
};

int JOYSTICK_CURRENT_STATE = STATE_BOOT;

// Calibration EEPROM addresses
#define ADDR_X_AXIS_PWM_TARGET 1
#define ADDR_X_AXIS_CAL_MAX 4
#define ADDR_X_AXIS_CAL_MIN 8
#define ADDR_X_AXIS_CURRENT 12
#define ADDR_Y_AXIS_PWM_TARGET 16
#define ADDR_Y_AXIS_CAL_MAX 20
#define ADDR_Y_AXIS_CAL_MIN 24
#define ADDR_Y_AXIS_CURRENT 28

//Change the default val to write new defaults on next boot
#define ADDR_SET_DEFAULT 0
#define ADDR_SET_DEFAULT_VAL 63

//Write int to EEPROM
void writeInt(int addr, int i)
{ 
  byte b1 = i >> 8;
  byte b2 = i & 0xFF;
  EEPROM.write(addr, b1);
  EEPROM.write(addr + 1, b2);
}

//Read int from EEPROM
int readInt(int addr)
{
  byte b1 = EEPROM.read(addr);
  byte b2 = EEPROM.read(addr + 1);
  return (b1 << 8) + b2;
}

//Calibration values
int x_axis_pwm_target = readInt(ADDR_X_AXIS_PWM_TARGET);
int x_axis_cal_max = readInt(ADDR_X_AXIS_CAL_MAX);
int x_axis_cal_min = readInt(ADDR_X_AXIS_CAL_MIN);
int x_axis_current = readInt(ADDR_X_AXIS_CURRENT);

int y_axis_pwm_target = readInt(ADDR_Y_AXIS_PWM_TARGET);
int y_axis_cal_max = readInt(ADDR_Y_AXIS_CAL_MAX);
int y_axis_cal_min = readInt(ADDR_Y_AXIS_CAL_MIN);
int y_axis_current = readInt(ADDR_Y_AXIS_CURRENT);

void set_default_eeprom() {
  writeInt(ADDR_X_AXIS_PWM_TARGET, 512);
  writeInt(ADDR_X_AXIS_CAL_MAX, 1023);
  writeInt(ADDR_X_AXIS_CAL_MIN, 0);
  writeInt(ADDR_X_AXIS_CURRENT, 5);
  writeInt(ADDR_Y_AXIS_PWM_TARGET, 512);
  writeInt(ADDR_Y_AXIS_CAL_MAX, 1023);
  writeInt(ADDR_Y_AXIS_CAL_MIN, 0);
  writeInt(ADDR_Y_AXIS_CURRENT, 5);
  EEPROM.write(ADDR_SET_DEFAULT, ADDR_SET_DEFAULT_VAL);
}

void debug_print_eeprom() {

  if (EEPROM.read(ADDR_SET_DEFAULT) != ADDR_SET_DEFAULT_VAL) {
    set_default_eeprom();
  }
  
  Serial.print("X_AX_PWM_TGT ");
  Serial.print(x_axis_pwm_target);
  Serial.print(" X_AX_CAL_MAX ");
  Serial.print(x_axis_cal_max);
  Serial.print(" X_AX_CAL_MIN ");
  Serial.print(x_axis_cal_min);
  Serial.print(" X_AX_I ");
  Serial.print(x_axis_current);
  Serial.print(" Y_AX_PWM_TGT ");
  Serial.print(y_axis_pwm_target);
  Serial.print(" Y_AX_CAL_MAX ");
  Serial.print(y_axis_cal_max);
  Serial.print(" Y_AX_CAL_MIN ");
  Serial.print(y_axis_cal_min);
  Serial.print(" Y_AX_I ");
  Serial.println(y_axis_current);
}

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//X-axis & Y-axis REQUIRED
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 4, 0,
  true, true, false, //X,Y,Z
  false, false, false,//Rx,Ry,Rz
  false, false, false, false, false);

Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[8] = {100};

float position_max[2] = {-100000};
float position_min[2] = {1000000};
float positions[2] = {0};

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

//This reads the X & Y axis encoders
volatile int pwm_x_value = 0;
volatile int prev_x_time = 0;
volatile int pwm_y_value = 0;
volatile int prev_y_time = 0;

void x_rising() {
  attachInterrupt(0, x_falling, FALLING);
  prev_x_time = micros();
}

void x_falling() {
  attachInterrupt(0, x_rising, RISING);
  pwm_x_value = micros() - prev_x_time;

  if (pwm_x_value > 1024) {
    pwm_x_value = 1024;
  }

  if (pwm_x_value < 0) {
    pwm_x_value = 0;
  }
}

void y_rising() {
  attachInterrupt(1, y_falling, FALLING);
  prev_y_time = micros();
}

void y_falling() {
  attachInterrupt(1, y_rising, RISING);
  pwm_y_value = micros() - prev_y_time;

  
  if (pwm_y_value > 1024) {
    pwm_y_value = 1024;
  }

  if (pwm_y_value < 0) {
    pwm_y_value = 0;
  }
}

void boot_initialize_pins() {
  Serial.println("BOOT: Init pins...");
  pinMode(6,OUTPUT);
  pinMode(10,OUTPUT);
}

void boot_initialize_calibration() {
  Serial.println("BOOT: Init cal...");
  debug_print_eeprom();
}

void boot_initialize_ffb() {
  Serial.println("BOOT: Init ffb...");
  Joystick.setXAxisRange(0, 1023);
  Joystick.setYAxisRange(0, 1023);
    //Steering wheel
    //Joystick.setXAxisRange(-512, 512);
    //set X Axis gains
    mygains[0].totalGain = 100;//0-100
    mygains[0].springGain = 50;//0-100
    mygains[1].totalGain = 100;
    mygains[1].springGain = 50;
    //enable gains REQUIRED
    Joystick.setGains(mygains);

    Joystick.begin(true);
}

void boot_connect_odrive() {
  Serial.println("BOOT: Init odrive...");
  odrive_serial.begin(115200);

}

void check_for_calibration_call() {
  Serial.println("cal call...");
  delay(3000);
  while(Serial.available() > 0 ){
    String str = Serial.readString();
    Serial.println(str);
    if(str.substring(0) == "cal\n"){
      Serial.println("iden");
      JOYSTICK_CURRENT_STATE = STATE_CALIBRATE;
      return;
    } else {
      Serial.println("nope");
    }
  }
  JOYSTICK_CURRENT_STATE = STATE_ODRIVE_INIT;
}

void setup(){
    positions[0] = GetPosition(0);
    positions[1] = GetPosition(1);

    Serial.begin(9600);
    // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
    attachInterrupt(0, x_rising, RISING); //pin 3
    attachInterrupt(1, y_rising, RISING); //pin 2
}

float GetPosition(int motor_number){
  odrive_serial << "r axis" << motor_number << ".encoder.pos_estimate\n";
  return odrive.readFloat();
}

float filterPosition(int motor_number) {
  float actual = GetPosition(motor_number);
  float local_max = position_max[motor_number]; 
  float local_min = position_min[motor_number];

  if (actual > local_max) {
    position_max[motor_number] = actual;
  }

  if (actual < local_min) {
    position_min[motor_number] = actual;
  }

  local_max = position_max[motor_number]; 
  local_min = position_min[motor_number];
//return range in 0 - 1023
  return ((actual - local_min) / (local_max - local_min));
}
  
void generic_loop() {

  //set X Axis Spring Effect Param
  myeffectparams[0].springMaxPosition = 1023;
  myeffectparams[0].springPosition = pwm_x_value;
  
  //set Y Axis Spring Effect Param
  myeffectparams[1].springMaxPosition = 1023;
  myeffectparams[1].springPosition = pwm_y_value;
  
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);


  float xForce = ((map(forces[0], -255, 255, 0, 1000) * .001f) * .3f) -.15f;
  float yForce = ((map(forces[1], -255, 255, 0, 1000) * .001f) * .3f) -.15f;

  Serial << "forces: " << forces[0] << ", " << forces[1] << " -> " << xForce << ", " << yForce << "\n";

  
  odrive_serial << "w axis" << 0 << ".controller.input_torque " << xForce << "\n";
  odrive_serial << "w axis" << 1 << ".controller.input_torque " << yForce << "\n";
  
  delay(1);

}

void calibrate_loop() {
  unsigned long start = millis();

  int x_max = 1025;
  int x_min = -1;
  int y_max = 1025;
  int y_min = -1;

  //360, 768
  //576, 700

  bool print_update = false;

  Serial.println("cal loop...");
  odrive.run_state(0, ODriveArduino::AXIS_STATE_IDLE, true);
  odrive.run_state(1, ODriveArduino::AXIS_STATE_IDLE, true);
  while (millis() < (start + 25000)) {

    print_update = false;

    if ((int)millis() % 500 == 0) {
      print_update = true;
    }

    if ((int)millis() % 100 == 0) {
      if (pwm_x_value < 550) {
        if (x_min < pwm_x_value) {
          x_min = pwm_x_value;
          print_update = true;
        }
      }

      if (pwm_x_value > 550) {
        if (x_max > pwm_x_value) {
          x_max = pwm_x_value;
          print_update = true;
        }
      }

      if (pwm_y_value < 635) {
        if (y_min < pwm_y_value) {
          y_min = pwm_y_value;
          print_update = true;
        }
      }

      if (pwm_y_value > 635) {
        if (y_max > pwm_y_value) {
          y_max = pwm_y_value;
          print_update = true;
        }
      }
    }

    if (print_update) {      
      Serial  << pwm_x_value << ", " << pwm_y_value << "-> " << "xVals: " << x_max << ", " << x_min << " yVals: " << y_max << ", " << y_min  << "\n";
    }

    delay(1);

  }


  writeInt(ADDR_X_AXIS_PWM_TARGET, ((x_max - x_min) / 2) + x_min);
  writeInt(ADDR_X_AXIS_CAL_MAX, x_max);
  writeInt(ADDR_X_AXIS_CAL_MIN, x_min);
  writeInt(ADDR_X_AXIS_CURRENT, 5);
  writeInt(ADDR_Y_AXIS_PWM_TARGET, ((y_max - y_min) / 2) + y_min);
  writeInt(ADDR_Y_AXIS_CAL_MAX, y_max);
  writeInt(ADDR_Y_AXIS_CAL_MIN, y_min);
  writeInt(ADDR_Y_AXIS_CURRENT, 5);

  x_axis_pwm_target = readInt(ADDR_X_AXIS_PWM_TARGET);
  x_axis_cal_max = readInt(ADDR_X_AXIS_CAL_MAX);
  x_axis_cal_min = readInt(ADDR_X_AXIS_CAL_MIN);
  x_axis_current = readInt(ADDR_X_AXIS_CURRENT);
  y_axis_pwm_target = readInt(ADDR_Y_AXIS_PWM_TARGET);
  y_axis_cal_max = readInt(ADDR_Y_AXIS_CAL_MAX);
  y_axis_cal_min = readInt(ADDR_Y_AXIS_CAL_MIN);
  y_axis_current = readInt(ADDR_Y_AXIS_CURRENT);

  JOYSTICK_CURRENT_STATE = STATE_IDLE;
}

void set_calibration_direction() {

  Serial << "Resetting Axis0&1\n";
  odrive.run_state(0, ODriveArduino::AXIS_STATE_IDLE, true);
  odrive.run_state(1, ODriveArduino::AXIS_STATE_IDLE, true);
  
  Serial.println("Setting X EncDir");

  while (x_axis_cal_min > pwm_x_value > 0 || 1024 > pwm_x_value > 990) {
    Serial << "waiting on x\n";
    delay(100);
  }
  odrive.run_state(1, ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH, true);
  Serial << "Done..\nSetting Y Enc dir";

  while (y_axis_cal_min > pwm_y_value > 0 || 1024 > pwm_y_value > 990) {
    Serial << "Waiting on y\n";
    delay(100);
  }
  Serial << "Done\n";
  odrive.run_state(0, ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH, true);
}

void start_odrive() {
  odrive.run_state(0, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true);
  odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true);
  Serial.println("Odrive started");
}

void loop(){
  switch(JOYSTICK_CURRENT_STATE) {
    case STATE_BOOT:
      boot_initialize_pins();
      boot_initialize_calibration();
      boot_initialize_ffb();
      boot_connect_odrive();
      JOYSTICK_CURRENT_STATE = STATE_IDLE;
      break;
      
    case STATE_IDLE:
      check_for_calibration_call();
      break;

    case STATE_ODRIVE_INIT:
      set_calibration_direction();
      start_odrive();
      JOYSTICK_CURRENT_STATE = STATE_GENERIC;
      break;
    
    case STATE_CALIBRATE:
      calibrate_loop();
      break;
      
    case STATE_DCS:
      //Not implemented
      break;
      
    case STATE_GENERIC:
      generic_loop();
      break;
      
    default:
      break;
  };
}

#include "BMI088.h"
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include <PWMServo.h>
#include <Math.h>

//PIN DECLARATIONS
#define red_light_pin 6
#define green_light_pin 7
#define blue_light_pin 8
#define pyro1 23
#define chipSelect 10

//PID VALUES
#define Pval 0.36
#define Ival 0.0
#define Dval 0.0

//CONSTANT VARIABLES
#define mech_ejec_var 90
#define apogee_delay 20
#define mount_offset 3.8
#define lift_accel -17.5
#define serv_x_pin 4
#define serv_y_pin 5
#define serv_e_pin 22
#define serv_offset_x -5
#define serv_offset_y 6
#define sweep_number 8
#define servo_sweep 24
#define sign -1

//INT VARIABLES
int SM_var;

//FLOAT VARIABLES
float dt;
float timeChange;
float yaw;
float pitch;
float roll;
float Yr;
float Xr;
float Zr;
float Ax;
float Ay;
float Az;
float yawR;
float pitchR;
float rollR;
float PID_int_x;
float PID_int_y;
float proportional_x;
float proportional_y;
float integral_x;
float integral_y;
float derivative_x;
float derivative_y;
float setpoint_x;
float setpoint_y;
float error_x = 0;
float error_y = 0;
float con_y;
float con_x;

//TIME VARIABLES
unsigned long timeNow;
unsigned long timeLast = 0;
unsigned long millisNow;
unsigned long state_time;
unsigned long ap_time;
unsigned long launch_time;

//QUAT ARRAYS
float Sw[4];
float Q_ori[4] = {1, 0, 0, 0};
float Q_d[4];
float norm;

//SERVOS
PWMServo servo_x;
PWMServo servo_y;
PWMServo servo_e;


//INITIALIZATION
Bmi088Accel accel(Wire, 0x19);
Bmi088Gyro gyro(Wire, 0x69);

void setup() {
  int status;
  Serial.begin(115200);
  Serial.println("START");
  pinMode(chipSelect, OUTPUT);
  servo_x.attach(serv_x_pin);
  servo_y.attach(serv_y_pin);
  servo_e.attach(serv_e_pin);
  pinMode(pyro1, OUTPUT);
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  delay(500);
}

void loop() {
  switch (SM_var) {
    case 0:         // start up and servo sweep
      RGB_color(255, 0, 0); // Red
      servo_test();
      SM_var = 1;
      break;

    case 1:         //idle
      RGB_color(0, 255, 0); // Green
      accel_output();
      pad_idle();
      break;

    case 2:         //tvc and apogee detection
      RGB_color(0, 255, 255); // Cyan
      gyro_output();
      angle_output();
      PID_calculate();
      servo_write();
      apogee();
      break;

    case 3:         //auto abort
      break;

    case 4:         //chute deployment
      RGB_color(255, 255, 125); // Raspberry
      chute_deploy();
      break;

    case 5:         //landing
      break;
  }
}
void gyro_output() {
  gyro.readSensor();
  Yr = gyro.getGyroY_rads();
  Xr = gyro.getGyroX_rads();
  Zr = gyro.getGyroZ_rads();
}
void accel_output() {
  accel.readSensor();
  Ax = accel.getAccelX_mss();
  Ay = accel.getAccelY_mss();
  Az = accel.getAccelZ_mss();

}
void set_PID() {
  yaw = 0;
  pitch = 0;
  roll = 0;
  integral_x = 0;
  integral_y = 0;
}
void angle_output() {
  timeNow = micros();
  timeChange = (timeNow - timeLast);
  dt = timeChange / 1000000;
  timeLast = timeNow;
  Sw[0] = 0; Sw[1] = Xr; Sw[2] = Yr; Sw[3] = Zr;

  Q_d[0] = (-0.5 * Q_ori[1] * Sw[1] - 0.5 * Q_ori[2] * Sw[2] - 0.5 * Q_ori[3] * Sw[3]);
  Q_d[1] = (0.5 * Q_ori[0] * Sw[1] + 0.5 * Q_ori[2] * Sw[3] - 0.5 * Q_ori[3] * Sw[2]);
  Q_d[2] = (0.5 * Q_ori[0] * Sw[2] - 0.5 * Q_ori[1] * Sw[3] + 0.5 * Q_ori[3] * Sw[1]);
  Q_d[3] = (0.5 * Q_ori[0] * Sw[3] + 0.5 * Q_ori[1] * Sw[2] - 0.5 * Q_ori[2] * Sw[1]);

  Q_ori[0] = Q_ori[0] + Q_d[0] * dt;
  Q_ori[1] = Q_ori[1] + Q_d[1] * dt;
  Q_ori[2] = Q_ori[2] + Q_d[2] * dt;
  Q_ori[3] = Q_ori[3] + Q_d[3] * dt;

  norm = sqrt(Q_ori[0] * Q_ori[0] + Q_ori[1] * Q_ori[1] + Q_ori[2] * Q_ori[2] + Q_ori[3] * Q_ori[3]);
  Q_ori[0] = Q_ori[0] / norm;
  Q_ori[1] = Q_ori[1] / norm;
  Q_ori[2] = Q_ori[2] / norm;
  Q_ori[3] = Q_ori[3] / norm;

  float y = atan2(2.0f * (Q_ori[1] * Q_ori[2] + Q_ori[0] * Q_ori[3]), Q_ori[0] * Q_ori[0] + Q_ori[1] * Q_ori[1] - Q_ori[2] * Q_ori[2] - Q_ori[3] * Q_ori[3]);
  float p = -asin(2.0f * (Q_ori[1] * Q_ori[3] - Q_ori[0] * Q_ori[2]));
  float r = atan2(2.0f * (Q_ori[0] * Q_ori[1] + Q_ori[2] * Q_ori[3]), Q_ori[0] * Q_ori[0] - Q_ori[1] * Q_ori[1] - Q_ori[2] * Q_ori[2] + Q_ori[3] * Q_ori[3]);


  yaw = y * 180.0f / PI;
  pitch = p * 180.0f / PI;
  roll = r * 180.0f / PI;

    Serial.print(Yr);  Serial.print("\t");  Serial.print(Xr);  Serial.print("\t");  Serial.print(Zr);  Serial.print("\t");
    ///Serial.print("Q_ori = ");  Serial.print(Q_ori[0]);  Serial.print("\t");  Serial.print(Q_ori[1]);  Serial.print("\t");  Serial.print(Q_ori[2]);  Serial.print("\t");  Serial.print(Q_ori[3]);  Serial.print("\t");
  //  Serial.print("Q_d = ");  Serial.print(Q_d[0]);  Serial.print("\t");  Serial.print(Q_d[1]);  Serial.print("\t");  Serial.print(Q_d[2]);  Serial.print("\t");  Serial.print(Q_d[3]);  Serial.print("\t");
    Serial.print("yaw = ");  Serial.print(yaw);  Serial.print("\t");  Serial.print("pitch = ");  Serial.print(pitch);  Serial.print("\t");  Serial.print("roll = ");  Serial.print(roll);  Serial.print("\t");
    Serial.println(dt, 5);
}
void PID_calculate() {
  proportional_x = (roll * Pval);
  proportional_y = (pitch * Pval);

  error_x = roll + error_x; integral_x = error_x * Ival;
  error_y = pitch + error_y; integral_y = error_y * Ival;

  float rate_deg_x = (Xr * 180 / PI); derivative_x = rate_deg_x * Dval;
  float rate_deg_y = (Yr * 180 / PI); derivative_y = rate_deg_y * Dval;

  PID_int_x = (proportional_x + integral_x + derivative_x) * sign;
  PID_int_y = (proportional_y + integral_y + derivative_y) * sign;
}
void servo_write() {
  float serv_int_x = (PID_int_x * mount_offset);
  con_x = constrain (-serv_int_x, -24, 24);
  float servoWrite_x = (con_x + 90 + serv_offset_x);

  float serv_int_y = (PID_int_y * mount_offset);
  con_y = constrain (serv_int_y, -24, 24);
  float servoWrite_y = (con_y + 90 + serv_offset_y);

  servo_x.write(servoWrite_x);
  servo_y.write(servoWrite_y);
  //  Serial.print("\t");  Serial.print(con_x);  Serial.print("\t");  Serial.println(con_y);
}
void pad_idle() {
  if (Az < lift_accel) {
    set_PID();
    timeLast = micros();
    launch_time = millis();
    SM_var = 2;
  }
 // Serial.print(Az);
 // Serial.print("    ");
 // Serial.println(lift_accel);
}
void apogee() {
  unsigned long now_time = millis();
  if ((now_time - launch_time) > (apogee_delay * 1000)) {
    SM_var = 4;
  }
}
void chute_deploy() {
  digitalWrite(pyro1, HIGH);
  servo_e.write(mech_ejec_var);
  SM_var = 5;
}

void servo_test() {
  for (int i = 0; i < sweep_number; i++) {
    servo_y.write(90 + serv_offset_y + servo_sweep);
    delay(50);
    servo_x.write(90 + serv_offset_x + servo_sweep);
    delay(50);
    servo_y.write(90 + serv_offset_y);
    delay(50);
    servo_y.write(90 + serv_offset_y - servo_sweep);
    delay(50);
    servo_x.write(90 + serv_offset_x);
    delay(50);
    servo_x.write(90 + serv_offset_x - servo_sweep);
    delay(50);
    servo_y.write(90 + serv_offset_y);
    delay(50);
    servo_y.write(90 + serv_offset_y + servo_sweep);
    delay(50);
    servo_x.write(90 + serv_offset_x);
    servo_y.write(90 + serv_offset_y);
  }
}
void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Servo esc_fl;
Servo esc_fr;
Servo esc_bl;
Servo esc_br;

#define ANGULAR_VELOCITY_P 0
#define ANGULAR_VELOCITY_D 0
#define ANGULAR_VELOCITY_I 0
#define ANGULAR_POSITION_P 0
#define ANGULAR_POSITION_D 0
#define ANGULAR_POSITION_I 0

#define MAX_ESC_OUT 1950
#define MIN_ESC_OUT 1050

#define INTERRUPT_PIN 0
#define LED_PIN 13

struct pid {
  float p_gain;
  float d_gain;
  float i_gain;

  float x_setpoint;
  float y_setpoint;
  float z_setpoint;

  float x_error;
  float x_error_prev;
  float x_current;
  float y_error;
  float y_error_prev;
  float y_current;
  float z_error;
  float z_error_prev;
  float z_current;

  float d_error_x;
  float p_error_x;
  float i_error_x;
  float d_error_y;
  float p_error_y;
  float i_error_y;
  float d_error_z;
  float p_error_z;
  float i_error_z;

  float x_output;
  float y_output;
  float z_output;

  unsigned long time_prev;
  unsigned long time_elapsed;
  bool first_time;
};
typedef struct pid PID;


MPU6050* mpu_ = &mpu;







int throttle = 1150;
int esc_fl_out = 0, esc_fr_out = 0, esc_bl_out = 0, esc_br_out = 0;

String input_string = "";
bool string_complete = false;

volatile bool mpuInterrupt = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


















void setup() {

  Wire.setSDA(18);
  Wire.setSCL(19);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(38400);
  Wire.begin();
  Wire.setClock(400000);


  // wait for ready
  Serial.println("Send any key to start up");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  esc_fl.attach(7, 1000, 2000);
  esc_fr.attach(8, 1000, 2000);
  esc_bl.attach(9, 1000, 2000);
  esc_br.attach(10, 1000, 2000);

  if (digitalRead(23) == HIGH) {
    calibrateESC();
  }
  writeESC(0, 0, 0, 0);
  input_string.reserve(100);

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();


  mpu.setXGyroOffset(159);
  mpu.setYGyroOffset(34);
  mpu.setZGyroOffset(31);
  mpu.setZAccelOffset(956);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println("Starting loop");


  PID angular_velocity = {ANGULAR_VELOCITY_P , ANGULAR_VELOCITY_D, ANGULAR_VELOCITY_I,
                          0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0,
                          0, 0,
                          false
                         };


  PID angular_position = {ANGULAR_POSITION_P , ANGULAR_POSITION_D, ANGULAR_POSITION_I,
                          0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0,
                          0, 0,
                          false
                         };

PID* angular_vel_ptr = &angular_velocity;
PID* angular_pos_ptr = &angular_position;



}

void loop() {
  // put your main code here, to run repeatedly:

}

void dmpDataReady() {
  mpuInterrupt = true;
}



void PID_Loop(PID* data) {

  data.elapsed






}

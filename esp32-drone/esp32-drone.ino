#include <ESP32Servo.h>
#include <RF24.h>
#include <PIDControl.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"

//#include <Kalman.h>
//#include "MPU6050.h"
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//#include <MPU6050_light.h>

#define DEBUG 1
#define LED 13

#define PIN_CH1 2
#define PIN_CH2 15
#define PIN_CH3 25
#define PIN_CH4 32
#define PIN_CH5 33
#define PIN_CH6 34
#define PIN_CH7 34


//---------------------------------------------------------
// FS-IA10B receiver setup - reading pulses with interruption
//---------------------------------------------------------

//CH1
volatile unsigned long pulseInTimeBegin_CH1 = micros();
volatile unsigned long pulseInTimeEnd_CH1 = micros();
volatile bool newPulseDurationAvailable_CH1 = false;
unsigned long int pulseDuration_CH1 = 0;

//CH2
volatile unsigned long pulseInTimeBegin_CH2 = micros();
volatile unsigned long pulseInTimeEnd_CH2 = micros();
volatile bool newPulseDurationAvailable_CH2 = false;
unsigned long int pulseDuration_CH2 = 0;

//CH3
volatile unsigned long pulseInTimeBegin_CH3 = micros();
volatile unsigned long pulseInTimeEnd_CH3 = micros();
volatile bool newPulseDurationAvailable_CH3 = false;
unsigned long int pulseDuration_CH3 = 0;

//CH4
volatile unsigned long pulseInTimeBegin_CH4 = micros();
volatile unsigned long pulseInTimeEnd_CH4 = micros();
volatile bool newPulseDurationAvailable_CH4 = false;
unsigned long int pulseDuration_CH4 = 0;

void receiverPinInterrupt_CH1(void)
{
  if (digitalRead(PIN_CH1) == HIGH) {
    // start measuring
    pulseInTimeBegin_CH1 = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd_CH1 = micros();
    newPulseDurationAvailable_CH1 = true;
  }
}

void receiverPinInterrupt_CH2(void)
{
  if (digitalRead(PIN_CH2) == HIGH) {
    // start measuring
    pulseInTimeBegin_CH2 = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd_CH2 = micros();
    newPulseDurationAvailable_CH2 = true;
  }
}

void receiverPinInterrupt_CH3(void)
{
  if (digitalRead(PIN_CH3) == HIGH) {
    // start measuring
    pulseInTimeBegin_CH3 = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd_CH3 = micros();
    newPulseDurationAvailable_CH3 = true;
  }
}

void receiverPinInterrupt_CH4(void)
{
  if (digitalRead(PIN_CH4) == HIGH) {
    // start measuring
    pulseInTimeBegin_CH4 = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd_CH4 = micros();
    newPulseDurationAvailable_CH4 = true;
  }
}

unsigned long readPulseCH1 (void)
{
  static unsigned long pulse_duration_last = 0;
  if (newPulseDurationAvailable_CH1) 
  {
    newPulseDurationAvailable_CH1 = false;
    pulseDuration_CH1 = pulseInTimeEnd_CH1 - pulseInTimeBegin_CH1;

    if (pulseDuration_CH1 > 2000) pulseDuration_CH1 = pulse_duration_last;
    else
      pulse_duration_last = pulseDuration_CH1;
    
  }
  
    return pulseDuration_CH1;
}

unsigned long readPulseCH2 (void)
{
    if (newPulseDurationAvailable_CH2) 
    {
      newPulseDurationAvailable_CH2 = false;
      pulseDuration_CH2 = pulseInTimeEnd_CH2 - pulseInTimeBegin_CH2;
    }
    return pulseDuration_CH2;
}

unsigned long readPulseCH3 (void)
{
  static unsigned long pulse_duration_last = 0;
  
  if (newPulseDurationAvailable_CH3) 
  {
    newPulseDurationAvailable_CH3 = false;
    pulseDuration_CH3 = pulseInTimeEnd_CH3 - pulseInTimeBegin_CH3;

    if (pulseDuration_CH3 > 2000) pulseDuration_CH3 = pulse_duration_last;
    else
      pulse_duration_last = pulseDuration_CH3;
    
  }
  
    return pulseDuration_CH3;
}

unsigned long readPulseCH4 (void)
{
    if (newPulseDurationAvailable_CH4) 
    {
      newPulseDurationAvailable_CH4 = false;
      pulseDuration_CH4 = pulseInTimeEnd_CH4 - pulseInTimeBegin_CH4;
    }
    return pulseDuration_CH4;
}

void initPinsFSI10AB (void)
{
  pinMode (PIN_CH1, INPUT);
  pinMode (PIN_CH2, INPUT);
  pinMode (PIN_CH3, INPUT);
  pinMode (PIN_CH4, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_CH1), receiverPinInterrupt_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH2), receiverPinInterrupt_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH3), receiverPinInterrupt_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH4), receiverPinInterrupt_CH4, CHANGE);
}

//---------------------------------------------------------


//---------------------------------------------------------
// RADIO NRF24L01
//---------------------------------------------------------

#define PAYLOAD_LENGTH 10

// radio 0 usa o endereco 0 pra transmitir (1node)
// radio 1 usa o enredeco 1 pra transmitir (2node)
#define RADIO_ID 1

// pinos: CE = 5, CSN = 4
RF24 radio(5, 4); 

// enderecos para leitura/escrita
uint8_t address[][6] = { "1Node", "2Node" };

// modo: TX = true, RX = false
bool modo = false;  

// mensagem (3 floats: PITCH, ROLL e YAW)
float payload[PAYLOAD_LENGTH] = {0, 0, 0};

void initRadio (void)
{
  if (!radio.begin()) 
  {
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(payload));

  radio.openWritingPipe(address[RADIO_ID]);  
  radio.openReadingPipe(1, address[!RADIO_ID]);  // using pipe 1

  //radio.stopListening();  // put radio in TX mode
  radio.startListening();  // put radio in RX mode
}

int sendDataRadio (float *data_out)
{
  int ret;
  
  radio.stopListening();

  ret = radio.write(data_out, sizeof(float) * PAYLOAD_LENGTH); 
  
  return (! ret);
}

int getDataRadio (float *data_in)
{
  int timeout = 50;
  
  radio.startListening();

  do
  {
    if (radio.available())
    {
      radio.read(data_in, sizeof (float) * PAYLOAD_LENGTH );             // fetch payload from FIFO
      //Serial.println("A: "+ String(payload[0]) + "    B: " + String(payload[1]) + "   C: " + String (payload[2]));

      radio.stopListening();
      return 1;
    }

    else
      timeout--;

  } while (timeout);
  

  return 0;
}
//---------------------------------------------------------


//--------------------------------------------------------------
// CONTROL
//--------------------------------------------------------------

Servo bldc1;
Servo bldc2;
Servo bldc3;
Servo bldc4;

int minUs = 1000;
int maxUs = 2000;


#define BLDC1_PIN 12  //CCW
#define BLDC3_PIN 27  //CCW

#define BLDC2_PIN 14  //CW
#define BLDC4_PIN 26  //CW

void CalibrateESC (void)
{
  bldc1.writeMicroseconds (maxUs);
  bldc2.writeMicroseconds (maxUs);
  bldc3.writeMicroseconds (maxUs);
  bldc4.writeMicroseconds (maxUs);
  blinkLed (LED, 20, 100);

  bldc1.writeMicroseconds (minUs);
  bldc2.writeMicroseconds (minUs);
  bldc3.writeMicroseconds (minUs);
  bldc4.writeMicroseconds (minUs);
  blinkLed (LED, 20, 100);
}

void initMotorControl (void)
{
    
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  bldc1.setPeriodHertz(50);      // Standard 50hz servo
  bldc2.setPeriodHertz(50);      // Standard 50hz servo
  bldc3.setPeriodHertz(50);      // Standard 50hz servo
  bldc4.setPeriodHertz(50);      // Standard 50hz servo


  pinMode(BLDC1_PIN, OUTPUT); 
  pinMode(BLDC2_PIN, OUTPUT);
  pinMode(BLDC3_PIN, OUTPUT); 
  pinMode(BLDC4_PIN, OUTPUT); 

  
  bldc1.attach (BLDC1_PIN, minUs, maxUs);
  bldc2.attach (BLDC2_PIN, minUs, maxUs);
  bldc3.attach (BLDC3_PIN, minUs, maxUs);
  bldc4.attach (BLDC4_PIN, minUs, maxUs);

  //CalibrateESC ();
  
  bldc1.writeMicroseconds (minUs);
  bldc2.writeMicroseconds (minUs);
  bldc3.writeMicroseconds (minUs);
  bldc4.writeMicroseconds (minUs);

  blinkLed (LED, 10, 100);
  
  
  return;

}
//--------------------------------------------------------------


//---------------------------------------------------------
// MPU6050 & Kalman
//---------------------------------------------------------
//MPU6050 mpu(Wire);
//Adafruit_MPU6050 mpu;
//sensors_event_t a, g, temp;
MPU6050 mpu;

//Kalman kalmanX; // roll
//Kalman kalmanY; // pitch

float dt, currentTime, previousTime;

int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll, angle_yaw;
float angle_pitch_acc, angle_roll_acc;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/*
void initImu (void)
{

  digitalWrite (LED, LOW);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0)
  { 
    blinkLed (LED, 1, 300);
  } 

  mpu.calcOffsets(true,true); // gyro and accelero
  
  // average calcuation to get gyro offset
  for (int i = 0; i < 1000; i++)
  {
    mpu.update();
    gyro_x_cal += mpu.getGyroX();
    gyro_y_cal += mpu.getGyroY();
    gyro_z_cal += mpu.getGyroZ();
    
    delay (1);
  }

  // calculates the average
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  kalmanX.setAngle (mpu.getAngleX());
  kalmanY.setAngle (mpu.getAngleY());

  digitalWrite (LED, HIGH);
  
  loop_timer = micros();
  
  return;
}
*/


void initImu (void)
{

  // init vars
//  acc_x = 0;
//  acc_y = 0;
//  acc_z = 0;

//  gyro_x = 0;
//  gyro_y = 0;
//  gyro_z = 0;

  angle_pitch = 0;
  angle_roll = 0;
  angle_yaw = 0;

  mpu.initialize();

  if (! mpu.testConnection())
  while(true)
  { 
    blinkLed (LED, 1, 300);
  } 

  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
  //  Serial.println();
//    mpu.PrintActiveOffsets();
  
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  } 
  else 
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  
/*  //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++)
  {                  

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    { // Get the Latest packet 

      mpu.dmpGetGyro(&gy, fifoBuffer);
     
      //Add the gyro x offset to the gyro_x_cal variable                                            
      gyro_x_cal += gy.x;
      //Add the gyro y offset to the gyro_y_cal variable                                              
      gyro_y_cal += gy.y; 
      //Add the gyro z offset to the gyro_z_cal variable                                             
      gyro_z_cal += gy.z; 
    
      //Delay 3us to have 250Hz for-loop                                             
      delay(3);                                                          
    }
  }
 
  // calculates the average
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  kalmanX.setAngle (0);
  kalmanY.setAngle (0);

*/
  digitalWrite (LED, HIGH);

  loop_timer = micros();
  
  return;

  
}

void getImuAngles (void)
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { // Get the Latest packet 

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    angle_pitch = ypr[1] * 180 / M_PI;
    angle_roll = ypr[2] * 180 / M_PI;
     
    Serial.print (angle_pitch); Serial.print ("\t");
    Serial.println (angle_roll);
  }
}

 




/*
 * //mpu6050_light
void getImuData (void)
{

  mpu.update();

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = micros();            // Current time actual time read
  dt = (currentTime - previousTime) / 1000000; // Divide by 1000000 to get seconds

  //Serial.println (dt, 11);

  
  gyro_x = mpu.getGyroX();
  gyro_y = mpu.getGyroY();
  gyro_z = mpu.getGyroZ();
  
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  
   // Kalman filter
  angle_roll = kalmanX.getAngle (mpu.getAccAngleX(), gyro_x, dt);
  angle_pitch = kalmanY.getAngle (mpu.getAccAngleY(), gyro_y, dt);

  angle_yaw = mpu.getAngleZ ();

  return;
}
*/

/*
 //Adafruit_mpu6050
void getImuData (void)
{

  mpu.getEvent(&a, &g, &temp);

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = micros();            // Current time actual time read
  dt = (currentTime - previousTime) / 1000000; // Divide by 1000000 to get seconds

  gyro_x = g.gyro.x;
  gyro_y = g.gyro.y;
  gyro_z = g.gyro.z;
  
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  
   // Kalman filter
  angle_roll = kalmanX.getAngle (a.acceleration.x, gyro_x, dt);
  angle_pitch = kalmanY.getAngle (a.acceleration.y, gyro_y, dt);

  angle_yaw = 0;

  return;
}
*/
//--------------------------------------------------------------


//--------------------------------------------------------------
// PID INIT
//--------------------------------------------------------------
float set_point_pitch;
float set_point_roll;
float set_point_yaw;
float kp = 0;
float ki = 0;
float kd = 0;

float kpy = 0;
float kiy = 0;
float kdy = 0;

float error;
float int_error;
float last_error;

float outputPitchPID;
float outputRollPID;
float outputYawPID;

PIDControl pitchPID;
PIDControl rollPID;
PIDControl yawPID;

void initPID (void)
{
  set_point_pitch = 0;
  set_point_roll = 0;
  set_point_yaw = 0;

//  kp = 10;
//  ki = 0.001;
//  kd = 3;

  kpy = 5;
  kiy = 0.015;
  kdy = 10;

  error = 0;
  int_error = 0;

  rollPID.begin (kp, ki, kd);
  pitchPID.begin (kp, ki, kd);
  yawPID.begin (kpy, kiy, kdy);

  //pitchPID.setOutputLimits (-400.0, 400.0);
  //rollPID.setOutputLimits (-400.0, 400.0);
  //yawPID.setOutputLimits (-400.0, 400.0);

}
//--------------------------------------------------------------

//--------------------------------------------------------------
// blink led
//--------------------------------------------------------------
void blinkLed (int led_pin, int times, int delay_ms)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite (led_pin, LOW);
    delay (delay_ms);
    digitalWrite (led_pin, HIGH);
    delay (delay_ms);
  }
}

//--------------------------------------------------------------



//--------------------------------------------------------------

void setup() 
{
  
  Serial.begin (115200);
  Wire.begin();
  pinMode (LED, OUTPUT);


  initPinsFSI10AB ();
  initImu ();
  initPID ();
  initMotorControl ();
  initRadio ();
}
//--------------------------------------------------------------


int set_running = 0;
float throttle_in = 0;
  
void loop() 
{

  int esc1;
  int esc2;
  int esc3;
  int esc4;
  

  throttle_in = readPulseCH3 ();

/*
  if (Serial.available()) 
  {
    throttle_in = Serial.parseFloat();

    // ignora o '\n'
    if (Serial.available())
      Serial.read();
  }
  
*/
  getImuAngles ();

  set_point_pitch = 0;
  set_point_roll = 0;
  
  
  if (throttle_in >= 1100)
  {
    set_running = 1;

//set_point_roll = mapf (readPulseCH1 (), 1000, 2000, -15, 15);
  //set_point_pitch = map (readPulseCH2 (), 1000, 2000, -30, 30);
  //set_point_yaw += mapf (readPulseCH4 (), 1000, 2000, -0.1, 0.1);

  
  outputPitchPID = pitchPID.computePID (angle_pitch, set_point_pitch);
  outputRollPID  = rollPID.computePID (angle_roll , set_point_roll);
  //outputYawPID  = yawPID.computePID (yaw , set_point_yaw);


  //esc1 = throttle_in + outputPitchPID;  // front right ccw
  //esc2 = throttle_in + outputPitchPID;  // front left cw
  //esc3 = throttle_in - outputPitchPID;  // rear left ccw 
  //esc4 = throttle_in - outputPitchPID;  // rear right cw
  

  //esc1 = throttle_in - outputRollPID;  // front right ccw
  //esc2 = throttle_in + outputRollPID;  // front left cw
  //esc3 = throttle_in + outputRollPID;  // rear left ccw 
  //esc4 = throttle_in - outputRollPID;  // rear right cw


  //esc1 = throttle_in -  outputYawPID;  // front right ccw
  //esc4 = throttle_in + outputYawPID;  // rear right cw
  //esc2 = throttle_in + outputYawPID;  // front left cw
  //esc3 = throttle_in - outputYawPID;  // rear left ccw 


    //esc1 = throttle_in - outputPitchPID - outputRollPID - outputYawPID;  // front right ccw
  //  esc2 = throttle_in - outputPitchPID + outputRollPID + outputYawPID;  // front left cw
//  esc3 = throttle_in + outputPitchPID + outputRollPID - outputYawPID;  // rear left ccw 
//    esc4 = throttle_in + outputPitchPID - outputRollPID + outputYawPID;  // rear right cw

    

  esc1 = throttle_in + outputPitchPID - outputRollPID; // front right ccw
  esc2 = throttle_in + outputPitchPID + outputRollPID;  // front left cw
  esc3 = throttle_in - outputPitchPID + outputRollPID;  // rear left ccw 
  esc4 = throttle_in - outputPitchPID - outputRollPID;  // rear right cw
  

    // 0 - 180 ou  1000 a 2000
   if (esc1 > 1800) esc1 = 1800;
    if (esc2 > 1800) esc2 = 1800;
    if (esc3 > 1800) esc3 = 1800;
    if (esc4 > 1800) esc4 = 1800;
  
    if (esc1 <= 1100) esc1 = 1100;
    if (esc2 <= 1100) esc2 = 1100;
    if (esc3 <= 1100) esc3 = 1100;
    if (esc4 <= 1100) esc4 = 1100;
    
  }
  else
  {
    esc1 = minUs;
    esc2 = minUs;
    esc3 = minUs;
    esc4 = minUs;
  }

  
  //---------------------------------------------------


  if (set_running == 1)
  {
    bldc1.writeMicroseconds (esc1);
    bldc2.writeMicroseconds (esc2);
    bldc3.writeMicroseconds (esc3);
    bldc4.writeMicroseconds (esc4);
  }
  else
  {
    bldc1.writeMicroseconds (1000);
    bldc2.writeMicroseconds (1000);
    bldc3.writeMicroseconds (1000);
    bldc4.writeMicroseconds (1000);
  }



//  Serial.print (angle_roll); Serial.print (" , ");
//  Serial.print (angle_pitch); Serial.print (" , ");
//  Serial.println (angle_yaw);


  payload[0] = angle_pitch;
  payload[1] = angle_roll;
  payload[2] = outputPitchPID;
  payload[3] = outputRollPID;
  payload[4] = kd;
  

  if (getDataRadio (payload))
  {
    kp = 1.2;
    ki = 0.002;
    kd = payload[0];

    //Serial.print ("Setting kp to: ");
    //Serial.println (kp, 5);

    pitchPID.setGains (kp, ki, kd);
    rollPID.setGains (kp, ki, kd);
  }

   sendDataRadio (payload);
  radio.startListening();
  
  
  
//-------------------------------------

  //Serial.println (String(micros() - loop_timer));
  while((micros() - loop_timer) < 10000);                                 
  loop_timer = micros();      
       
}

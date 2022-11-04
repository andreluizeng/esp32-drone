#include <PIDControl.h>
#include <Wire.h>
#include <KalmanFilter.h>
#include "Mapf.h"

#define DEBUG 1

#define LED 4

#define SENSOR_BAT 35

#define PIN_CH1 26
#define PIN_CH2 25
#define PIN_CH3 33
#define PIN_CH4 32
#define PIN_CH5 35
#define PIN_CH6 34

#define IMU_CAL_SIZE 4000

bool armed = 0;

//--------------------------------------------------------------
// blink led
//--------------------------------------------------------------
void blinkLed (int led_pin, int times, int delay_ms)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite (led_pin, HIGH);
    delay (delay_ms);
    digitalWrite (led_pin, LOW);
    delay (delay_ms);
  }
}

//--------------------------------------------------------------



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
    static unsigned long pulse_duration_last = 0;
    
    if (newPulseDurationAvailable_CH2) 
    {
      newPulseDurationAvailable_CH2 = false;
      pulseDuration_CH2 = pulseInTimeEnd_CH2 - pulseInTimeBegin_CH2;

      if (pulseDuration_CH2 > 2000) pulseDuration_CH2 = pulse_duration_last;
      
      else
        pulse_duration_last = pulseDuration_CH2;

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
    static unsigned long pulse_duration_last = 0;
    
    if (newPulseDurationAvailable_CH4) 
    {
      newPulseDurationAvailable_CH4 = false;
      pulseDuration_CH4 = pulseInTimeEnd_CH4 - pulseInTimeBegin_CH4;

    if (pulseDuration_CH4 > 2000) pulseDuration_CH4 = pulse_duration_last;
    else
      pulse_duration_last = pulseDuration_CH4;

    }
    return pulseDuration_CH4;
}

float ch1_zero = 0;
float ch2_zero = 0;
float ch4_zero = 0;

void initPinsFSI10AB (void)
{
  pinMode (PIN_CH1, INPUT_PULLUP);
  pinMode (PIN_CH2, INPUT_PULLUP);
  pinMode (PIN_CH3, INPUT_PULLUP);
  pinMode (PIN_CH4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_CH1), receiverPinInterrupt_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH2), receiverPinInterrupt_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH3), receiverPinInterrupt_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH4), receiverPinInterrupt_CH4, CHANGE);

  delay (1000);

  for (int i = 0; i < 100; i++)
  {
    ch4_zero += readPulseCH4 ();
    ch1_zero += readPulseCH1 ();
    ch2_zero += readPulseCH2 ();

    delay (20);
  }

  ch4_zero /= 100;
  ch1_zero /= 100;
  ch2_zero /= 100;

  ch4_zero = 1500 - ch4_zero;
  ch1_zero = 1500 - ch1_zero;
  ch2_zero = 1500 - ch2_zero;

}

//---------------------------------------------------------


//--------------------------------------------------------------
// BLDCS CONTROL
//--------------------------------------------------------------
#define BLDC1_PIN 13  //CW
#define BLDC2_PIN 12  //CCW
#define BLDC3_PIN 14  //CW
#define BLDC4_PIN 27   //CCW

int32_t mask_bldcs;

void InitEscs (void)
{
  pinMode (BLDC1_PIN, OUTPUT);
  pinMode (BLDC2_PIN, OUTPUT);
  pinMode (BLDC3_PIN, OUTPUT);
  pinMode (BLDC4_PIN, OUTPUT);
     
  mask_bldcs = (1ULL << BLDC1_PIN) | (1ULL << BLDC2_PIN) | (1ULL << BLDC3_PIN) | (1ULL << BLDC4_PIN);
  blinkLed (LED, 3, 100);
}

//--------------------------------------------------------------
// mask format: (1 << DESIRED_IO_1) | (1 << DESIRED_IO_2) | () and so on...
//--------------------------------------------------------------
void SetValueGPIOS (int32_t mask, bool value)
{
  if (value)
      GPIO.out_w1ts = mask;
  else
      GPIO.out_w1tc = mask;

  return;
}
//--------------------------------------------------------------

void SetEscValues (int esc_1, int esc_2, int esc_3, int esc_4)
{
  static unsigned long loop_timer = 0;
  unsigned long timer_esc1 = 0;
  unsigned long timer_esc2 = 0;
  unsigned long timer_esc3 = 0;
  unsigned long timer_esc4 = 0;

  unsigned long esc_loop_timer = 0;
  bool flag_esc1 = 1;
  bool flag_esc2 = 1;
  bool flag_esc3 = 1;
  bool flag_esc4 = 1;

  loop_timer = micros(); 

  SetValueGPIOS (mask_bldcs, HIGH);
  flag_esc1 = 1;
  flag_esc2 = 1;
  flag_esc3 = 1;
  flag_esc4 = 1;
  
  timer_esc1 = esc_1 + loop_timer;  
  timer_esc2 = esc_2 + loop_timer;  
  timer_esc3 = esc_3 + loop_timer;  
  timer_esc4 = esc_4 + loop_timer;  

  do
  {  
      esc_loop_timer = micros();

      if (timer_esc1 < esc_loop_timer)
      {
        flag_esc1 = 0;
        SetValueGPIOS (1 << BLDC1_PIN, LOW);
      }
      if (timer_esc2 < esc_loop_timer)
      {
        flag_esc2 = 0;
        SetValueGPIOS (1 << BLDC2_PIN, LOW);
      }
      if (timer_esc3 < esc_loop_timer)
      {
        flag_esc3 = 0;
        SetValueGPIOS (1 << BLDC3_PIN, LOW);
      }
      if (timer_esc4 < esc_loop_timer)
      {
        flag_esc4 = 0;
        SetValueGPIOS (1 << BLDC4_PIN, LOW);
      }
      
      yield ();
  } while (flag_esc1 | flag_esc2 | flag_esc3 | flag_esc4);
}
//--------------------------------------------------------------


//--------------------------------------------------------------


//---------------------------------------------------------
// MPU6050 & Kalman
//---------------------------------------------------------

float dt, currentTime, previousTime;

float angle_pitch, angle_roll, angle_yaw;
float angle_pitch_output, angle_roll_output;
//float gyroRoll, gyroPitch, gyroYaw;
//float accRoll, accPitch, accYaw;
float angle_roll_acc, angle_pitch_acc;
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
float acc_x_cal, acc_y_cal, acc_z_cal;
float temp_cal;
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;
int16_t temp;
int16_t acc_total_vector;


//KalmanFilter kalmanRoll(0.001, 0.003, 0.1);

//KalmanFilter kalmanPitch(0.001, 0.003, 0.1);


void readImu (int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *t)
{
    
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14) yield ();                                        
  (*ax) = Wire.read()<<8|Wire.read();                               
  (*ay) = Wire.read()<<8|Wire.read();                               
  (*az) = Wire.read()<<8|Wire.read();                                 
   (*t) = Wire.read()<<8|Wire.read();                           
  (*gx) = Wire.read()<<8|Wire.read();                                
  (*gy) = Wire.read()<<8|Wire.read();                                 
  (*gz) = Wire.read()<<8|Wire.read();                                              

  return;
}

void initImu (void)
{  

  gyro_x_cal = 0;
  gyro_y_cal = 0;
  gyro_z_cal = 0;

  acc_x_cal = 0;
  acc_y_cal = 0;
  acc_z_cal = 0;

  temp_cal = 0;

  angle_pitch = 0;
  angle_roll = 0;
  angle_yaw = 0;

  Wire.begin();
  Wire.setClock(400000);

  delay(1000);

  Wire.beginTransmission(0x68);  // endereco do mpu50                                      
  Wire.write(0x6B);              // endereco de power/sleep control                                       
  Wire.write(0x00);              // power mode                                   
  Wire.endTransmission();         
  
  Wire.beginTransmission(0x68);                                      
  Wire.write(0x1C);              // endereco de configuracao do ACC
  Wire.write(0x10);              // 0x00 = 2G, 0x01 = 4G, 0x10 = 8G, 0x11 = 16G
  Wire.endTransmission();             
  
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);              // endereco de configuracao do GYRO
  Wire.write(0x08);              // 0x08= 500DEG
  Wire.endTransmission();  

  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1A);              // CONFIG register (1A hex)
  Wire.write(0x03);              // 00000011 (Set Digital Low Pass Filter to ~43Hz)  
  Wire.endTransmission();  
  
  delay(300);


//  calibrateImu ();  

//Temp offset: -0.82
//acc offset(x,y,z): -30.46 139.00 4233.46
//gyro offset(x,y,z): -201.71 22.41 29.28




  //acc_x_cal = 37.74;
  //acc_y_cal = 86.41;
  //acc_z_cal = 6934.80;

  gyro_x_cal = -201.71;
  gyro_y_cal = 22.41;
  gyro_z_cal = 29.28;


  
  temp_cal = -0.82;

  return;
}


void calibrateImu (void)
{
  for (int i = 0; i < IMU_CAL_SIZE; i++)
  {
    readImu(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &temp);

    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;

    temp_cal = temp;

    delay (3);
  }

  gyro_x_cal /= IMU_CAL_SIZE;
  gyro_y_cal /= IMU_CAL_SIZE;
  gyro_z_cal /= IMU_CAL_SIZE;

  acc_x_cal /= IMU_CAL_SIZE;
  acc_y_cal /= IMU_CAL_SIZE;
  acc_z_cal /= IMU_CAL_SIZE;

  temp_cal /= IMU_CAL_SIZE;

  Serial.print ("Temp offset: "); Serial.println (temp_cal);
  Serial.print ("acc offset(x,y,z): "); Serial.println (String(acc_x_cal) + " " + String (acc_y_cal) + " " + String (acc_z_cal));
  Serial.print ("gyro offset(x,y,z): "); Serial.println (String (gyro_x_cal) + " " + String (gyro_y_cal) + " " + String (gyro_z_cal));

  delay (5000);

  return;
}


float gyro_roll_input = 0;
float gyro_pitch_input = 0;
float gyro_yaw_input = 0;
float roll_level_adjust, pitch_level_adjust;
boolean auto_level = true;

void getImuAngles (void)
{

  readImu(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &temp);

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //acc_x -= acc_x_cal;
  //acc_y -= acc_y_cal;
  //acc_z -= acc_z_cal;

  // cada loop = 2ms = 500Hz (refresh rate)
  // de acordo com o datasheet do mpu6050, com a config em  500º/s a saida é de 65.5 quando a velocidade angular é de 1º/s, 1/65.5 = 0.0152
  // entao a saida do gyro é valor_raw / refresh_rate / 65.5, em graus/s => angulo += raw / 500 / 65.5 = raw * 0.0000305; (0.0000611 para 250hz)

  angle_pitch -= gyro_y * 0.0000611; 
  angle_roll -= gyro_x * 0.0000611; 
  angle_yaw += gyro_z * 0.0000611; 

  // roll e pitch sao calculados de acordo com a composicao das rotacoes em x e y (funcao seno), o valor tem que ser convertido novamente pra radianos:
  // valor_rad = valor_graus * 0.0000305 * (3.14 / 180) -> valor_rad = valor_graus * 0.000000533 (0.000001066 para 250hz)

  // roll -= pitch * sin (gyroz rad); // pitch = angle_x
  // pitch += roll * sin (gyroz rad); // roll angle_y
  angle_pitch += angle_roll * sin (gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin (gyro_z * 0.000001066);

  acc_total_vector = sqrt ((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));

  if(abs(acc_x) < acc_total_vector)
  {                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_x/acc_total_vector)* 57.296;          // 57.296 = 3.14 / 180
  }
  if(abs(acc_y) < acc_total_vector)
  {                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_y/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }

  // calibrar acc aqui.
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.


  //angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  //angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;  

//  Serial.println("@A"+ String(angle_pitch) + "B" + String (angle_roll) + "#");

  return;
}


//--------------------------------------------------------------
// PID INIT
//--------------------------------------------------------------
float set_point_pitch;
float set_point_roll;
float set_point_yaw;

float set_point_dist_pitch;
float set_point_dist_roll;

float kp = 0;
float ki = 0;
float kd = 0;
float kp2 = 0;
float ki2 = 0;
float kd2 = 0;


float kp_y = 0;
float ki_y = 0;
float kd_y = 0;
float kp_y2 = 0;
float ki_y2 = 0;
float kd_y2 = 0;

float kp_dist = 0;
float ki_dist = 0;
float kd_dist = 0;



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



 // 1.5, 0.002, 270 = ok
  kp = 3.3;
  ki = 0.001;
  kd = 200;

  kp_y = 5;
  ki_y = 0.005;
  kd_y = 0;

  

  error = 0;
  int_error = 0;

  rollPID.begin (kp, ki, kd);
  pitchPID.begin (kp, ki, kd);
  yawPID.begin (kp_y, ki_y, kd_y);

  pitchPID.setOutputLimits (-400.0, 400.0);
  rollPID.setOutputLimits (-400.0, 400.0);
  yawPID.setOutputLimits (-400.0, 400.0);

  //pitchPID.setGains (kp, ki, kd);
  //rollPID.setGains (kp, ki, kd);
  //yawPID.setGains (kpy, kiy, kdy);


}
//--------------------------------------------------------------


//--------------------------------------------------------------

void setup() 
{

   Serial.begin (115200);
   
   pinMode (3, INPUT_PULLUP); // RX0  
   pinMode (LED, OUTPUT);
   
   blinkLed (LED, 3, 100);
   digitalWrite (LED, HIGH);

  //Serial.println ("OK");
  //Serial.print ("receiver initialized...");
  initPinsFSI10AB ();
  //Serial.println ("OK");
  //Serial.print ("imu initialized...");
  initImu ();
  //Serial.println ("OK");
  //Serial.print ("PID initialized...");
  initPID ();
  //Serial.println ("OK");
  //Serial.print ("Motors initialized...");

  InitEscs ();
  //Serial.println ("OK");
  //Serial.print ("Radio initialized...");
  //initRadio ();

  blinkLed (LED, 10, 100);

}
//--------------------------------------------------------------


int set_running = 0;
float throttle_in = 0;
bool reset = 0; 

int fall_count = 0;
float set_point_yaw_angle = 0;


float pterm, iterm, dterm;


long cTime = 0;
long lTime = 0;
long eTime = 0;

void loop() 
{

  int esc1;
  int esc2;
  int esc3;
  int esc4;
  
  getImuAngles ();

  int sensor_bat_value = analogRead(SENSOR_BAT);  // 2470 = bateria cheia, menor que 2000 bsateria baixa, menor que 1800 praticamente esgotada

  //Serial.println (sensor_bat_value);

  // PID variables
  throttle_in = readPulseCH3 ();
  long int pulsos_ch4 = readPulseCH4 ();
  long int pulsos_ch1 = readPulseCH1 ();
  long int pulsos_ch2 = readPulseCH2 ();

  pulsos_ch4 += (long int)ch4_zero;
  pulsos_ch1 += (long int)ch1_zero;
  pulsos_ch2 += (long int)ch2_zero;

  if ((pulsos_ch1 > 1493) && (pulsos_ch1 < 1507)) pulsos_ch1 = 1500;
  if ((pulsos_ch2 > 1493) && (pulsos_ch2 < 1507)) pulsos_ch2 = 1500;
  if ((pulsos_ch4 > 1493) && (pulsos_ch4 < 1507)) pulsos_ch4 = 1500;


  set_point_roll =  mapf (pulsos_ch1, 1000, 2000, 40, -40);
  set_point_pitch = mapf (pulsos_ch2, 1000, 2000, -40, 40); 
  set_point_yaw = mapf (pulsos_ch4, 1000, 2000, -70, 70);


  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  //gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_x / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  //gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_y / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //pitchPID.splitPID (&pterm, &iterm, &dterm);
//  Serial.println("@A"+ String(angle_pitch)+ "B"+ String(angle_roll) + "C" + String (gyro_yaw_input) + "#");

//    Serial.println("@A"+ String(outputYawPID)+ "B"+ String(outputRollPID) + "C" + String (outputPitchPID) + "#");

    //Serial.println("@A"+ String(outputPitchPID)+ "B"+ String(angle_pitch)+ "C"+ String(angle_roll)+"D"+ String(outputRollPID)+"#");
    //Serial.println (String(pulsos_ch4) + " "+ String(pulsos_ch1) + " " + String (pulsos_ch2)+ "---" + String(ch4_zero) + " "+ String(ch1_zero) + " " + String (ch2_zero));

 //Serial.println (throttle_in);

  if (Serial.available()) 
 {
    blinkLed (LED, 10, 40);
    kp = Serial.parseFloat(); //ki

    while (!Serial.available());
    ki = Serial.parseFloat(); //kd
    
    while (!Serial.available());
    kd = Serial.parseFloat(); //kpy

    rollPID.setGains (kp, ki, kd);
    pitchPID.setGains (kp, ki, kd);
    //yawPID.setGains (kp_y, ki_y, kd_y);
    
    Serial.read(); // '/n' do final
  }

//  if ((angle_roll > 150) || (angle_roll < -150) || (angle_pitch > 150) || (angle_pitch < -150))
//  {
//    armed = false;
//    set_running = 0;
//    reset = 1;

//    angle_pitch = 0;
//    angle_roll = 0;
//    angle_yaw = 0;
//    esc1 = esc2 = esc3 = esc4 = 1000;
//  }

//  else
//  {
    if ((throttle_in < 1050) && (pulsos_ch4  > 1800))
    {

      cTime = millis ();
      lTime = cTime;
  
      do
      {
        throttle_in = readPulseCH3 ();
        pulsos_ch4 = readPulseCH4 ();

        cTime = millis ();
    
        eTime = cTime - lTime;
    
        if (eTime > 1000) break;

        yield ();
     
      } while ((throttle_in < 1050) && (pulsos_ch4  > 1800));
  
      // menu ok  
      if (eTime > 1000)
      {
        armed = false;

        angle_pitch = 0;
        angle_roll = 0;
        angle_yaw = 0;
        
        reset = 0;
  
        pitchPID.resetErrors();
        rollPID.resetErrors();
        yawPID.resetErrors();
  
        digitalWrite (LED, HIGH);      

        esc1 = esc2 = esc3 = esc4 = 1000;
      }

    }

    if ((throttle_in < 1050) && (pulsos_ch4 < 1200))
    {
      cTime = millis ();
      lTime = cTime;

      do
      {
        throttle_in = readPulseCH3 ();
        pulsos_ch4 = readPulseCH4 ();

        cTime = millis ();
    
        eTime = cTime - lTime;
    
        if (eTime > 1000) break;

        yield ();
     
      } while ((throttle_in < 1050) && (pulsos_ch4  < 1200));
      
      if (eTime > 1000)
      {
        armed = true;
  
        pitchPID.resetErrors();
        rollPID.resetErrors();
        yawPID.resetErrors();
  
        set_point_yaw = 0;

        angle_pitch = 0;
        angle_roll = 0;
        angle_yaw = 0;

        digitalWrite (LED, LOW);
        delay (10);

        esc1 = esc2 = esc3 = esc4 = 1000;
      }
    }
    if (armed)
    {

      // se o reset estiver desativado, operacao normal
      if (! reset)
      {
        outputRollPID  = rollPID.computePID (angle_roll , set_point_roll);
        outputPitchPID = pitchPID.computePID (angle_pitch, set_point_pitch);
        outputYawPID  = yawPID.computePID (gyro_yaw_input, set_point_yaw);

        //esc1 = throttle_in + outputPitchPID;  // front right ccw
        //esc2 = throttle_in + outputPitchPID;  // front left cw
        //esc3 = throttle_in - outputPitchPID;  // rear left ccw 
        //esc4 = throttle_in - outputPitchPID;  // rear right cw
      
    
        //esc1 = throttle_in - outputRollPID;  // front right ccw
        //esc2 = throttle_in + outputRollPID;  // front left cw
        //esc3 = throttle_in + outputRollPID;  // rear left ccw 
        //esc4 = throttle_in - outputRollPID;  // rear right cw
    
    
        //esc1 = throttle_in +  outputYawPID;  // front right ccw
        //esc2 = throttle_in - outputYawPID;  // rear right cw
        //esc3 = throttle_in + outputYawPID;  // front left cw
        //esc4 = throttle_in - outputYawPID;  // rear left ccw 
    
        //esc1 = throttle_in + outputPitchPID - outputRollPID; // front right ccw
        //esc2 = throttle_in + outputPitchPID + outputRollPID;  // front left cw
        //esc3 = throttle_in - outputPitchPID + outputRollPID;  // rear left ccw 
        //esc4 = throttle_in - outputPitchPID - outputRollPID;  // rear right cw
     
        esc1 = throttle_in + outputPitchPID - outputRollPID + outputYawPID; // front right ccw
        esc2 = throttle_in + outputPitchPID + outputRollPID - outputYawPID; // rear right cw
        esc3 = throttle_in - outputPitchPID + outputRollPID + outputYawPID; //rear left ccw 
        esc4 = throttle_in - outputPitchPID - outputRollPID - outputYawPID; //front right cw

        
    
        // 0 - 180 ou  1000 a 2000
        if (esc1 > 2000) esc1 = 2000;
        if (esc2 > 2000) esc2 = 2000;
        if (esc3 > 2000) esc3 = 2000;
        if (esc4 > 2000) esc4 = 2000;
      
        //if (esc1 <= 1200) esc1 = 1200;
        //if (esc2 <= 1200) esc2 = 1200;
        //if (esc3 <= 1200) esc3 = 1200;
        //if (esc4 <= 1200) esc4 = 1200;

        if (throttle_in > 1100)
        {
          if (esc1 <= 1100) esc1 = 1100;
          if (esc2 <= 1100) esc2 = 1100;      
          if (esc3 <= 1100) esc3 = 1100;
          if (esc4 <= 1100) esc4 = 1100;
        }
        else
        {
          esc1 = esc2 = esc3 = esc4 = throttle_in;

          pitchPID.resetErrors();
          rollPID.resetErrors();
          yawPID.resetErrors();

        }
        
      }
      
    }

    // ao zerar o throttle, reinicia o sistema e zera o reset
    else
    {
      pitchPID.resetErrors();
      rollPID.resetErrors();
      yawPID.resetErrors();

      esc1 = esc2 = esc3 = esc4 = 1000;
      digitalWrite (LED, HIGH);

      reset = 0;
    }
//  }
  
  //---------------------------------------------------

  yield ();          

  SetEscValues (esc1, esc2, esc3, esc4);

  do 
  {
    currentTime = micros();
    dt = (currentTime - previousTime);// / (float) 1000000;
     yield ();          
  } while (dt < 4000);
  previousTime = currentTime;


}

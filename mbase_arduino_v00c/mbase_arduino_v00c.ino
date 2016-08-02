//****************************************************************//
// Mobile base controller
// Created: 25/06/2016 by Henrique Latorre
//****************************************************************//

//#include <string>
//#include <sstream>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <PID_v1.h>
#include "LeddarFix.h"
#include <Servo.h>

#include "Encoder.h"
#include "DCMotor.h"

//#define DEBUG_TEST_MSGS

// Pinout
#define PIN_ENCODER_L     2   // left wheel encoder input: use interrupt pins
#define PIN_ENCODER_R     3   // right wheel encoder input: use interrupt pins
#define PIN_MOTOR_DIR_L   4   // direction line for the H-bridge left wheel
#define PIN_MOTOR_PWM_L   5   // PWM line for the H-bridge left wheel
#define PIN_MOTOR_DIR_R   7   // direction line for the H-bridge right wheel
#define PIN_MOTOR_PWM_R   6   // PWM line for the H-bridge right wheel
#define PIN_BATTERY_V     A0  // Battery level sensing
#define PIN_LED           13
#define PIN_LEDDAR_RX     8
#define PIN_LEDDAR_TX     9
#define PIN_LEDDAR_TX_EN  10
#define PIN_LEDDAR_SERVO  11


// Timing and update rates
#define PERIOD_ENCODER_MS         100
#define PERIOD_ENCODER_PULSE_MS   10  // 15 works fine
#define PERIOD_IMU_MS             20
#define PERIOD_LIDAR_MS           100
#define MOTOR_PWM_DEAD_ZONE       100
// Movement threshold
#define MOTOR_MINIMUM_SPEED       0.01
#define ENCODER_COUNTS_PER_METER  181.00


// Serial protocol flags
#define MSG_END_BYTE        '\n'
#define MSG_FIELD_SEPARATOR ';'
#define MSG_ODOMETRY        'o'
#define MSG_IMU             'i'
#define MSG_LIDAR_READING   'l'
#define MSG_LIDAR_ANGLE     'a'
#define MSG_VELOCITY        'v'
#define MSG_TIMESTAMP       millis()
#define SERIAL_IN_BUFF_SZ   100

#define SERIAL_BAUD         115200
#define LEDDAR_BAUD         9600

#define FORWARD 1
#define REVERSE -1

// Global Stuff
String g_input_string = "";         // a string to hold incoming data
boolean g_string_complete = false;  // whether the string is complete

volatile int8_t g_motor_l_forward = true;
volatile int8_t g_motor_r_forward = true;

double g_l_setpoint, g_l_input, g_l_output;
double g_r_setpoint, g_r_input, g_r_output;

//SoftwareSerial SSerial(PIN_LEDDAR_RX, PIN_LEDDAR_TX); // RX, TX

//Baudrate = 9600 (not the default)
//Modbus slave ID = 01
//LeddarOne g_leddar( (unsigned long)LEDDAR_BAUD, (unsigned char)1, SSerial, (unsigned char)PIN_LEDDAR_TX_EN, (unsigned char)1);

Servo g_leddar_servo;

LSM6 g_imu;
LIS3MDL g_mag;

Encoder g_encoder_l(PERIOD_ENCODER_PULSE_MS);
Encoder g_encoder_r(PERIOD_ENCODER_PULSE_MS);

DCMotor g_motor_l(PIN_MOTOR_PWM_L, PIN_MOTOR_DIR_L);
DCMotor g_motor_r(PIN_MOTOR_PWM_R, PIN_MOTOR_DIR_R);

PID g_l_PID(&g_l_input, &g_l_output, &g_l_setpoint, 500,400,50, DIRECT);
PID g_r_PID(&g_r_input, &g_r_output, &g_r_setpoint, 500,400,50, DIRECT);
//PID g_r_PID(&g_r_input, &g_r_output, &g_r_setpoint, 1000,800,100, DIRECT);

void encoder_l_event() {
  //noInterrupts();
  detachInterrupt(0);
  g_encoder_l.pulse(g_motor_l_forward);
  attachInterrupt(0, encoder_l_event, CHANGE);
  //interrupts();
}

void encoder_r_event() {
  //noInterrupts();
  detachInterrupt(1);
  g_encoder_r.pulse(g_motor_r_forward);
  attachInterrupt(1, encoder_r_event, CHANGE);
  //interrupts();
}

void i2c_init(){

  //Initialize Leddar 
  //g_leddar.init();
  
  // I2C interface for IMU
  Wire.begin();

  // Interrupts for encoders
  //pinMode(PIN_ENCODER_L, INPUT_PULLUP);
  pinMode(PIN_ENCODER_L, INPUT);
  //pinMode(PIN_ENCODER_R, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R, INPUT);
  delay(10);
  attachInterrupt(0, encoder_l_event, CHANGE);
  attachInterrupt(1, encoder_r_event, CHANGE);

  if (!g_imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    //while (1);
  }
  g_imu.enableDefault();

  if (!g_mag.init()) {
    Serial.println("Failed to detect and initialize magnetometer!");
    //while (1);
  }
  g_mag.enableDefault();
  // reserve 200 bytes for the input_string:
  g_input_string.reserve(SERIAL_IN_BUFF_SZ);
}

void setup() {
  
  Serial.begin(SERIAL_BAUD);
  i2c_init();
  g_l_PID.SetOutputLimits(-255, 255);
  g_r_PID.SetOutputLimits(-255, 255);
  g_l_PID.SetSampleTime(50);
  g_r_PID.SetSampleTime(50);
  g_l_PID.SetMode(AUTOMATIC);
  g_r_PID.SetMode(AUTOMATIC);
  g_leddar_servo.attach(PIN_LEDDAR_SERVO);
}

bool encoder_update () {
  static uint32_t timer = 0;
  static uint32_t s_last_time;
  static int32_t s_l_last_read, s_r_last_read;
  if (uint32_t(millis()-timer) > PERIOD_ENCODER_MS){
    timer = millis();

#ifdef DEBUG_TEST_MSGS
    Serial.print(MSG_ODOMETRY);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print("12;34");
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);
    return true;
#endif
    Serial.print(MSG_ODOMETRY);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_encoder_l.read());
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_encoder_r.read());
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);
    return true;
  }
  return false;
}

bool speed_update () {
  static uint32_t timer = 0;
  uint32_t dt = (millis()-timer);
  static int32_t s_l_last_read, s_r_last_read;
  if (dt > 200){
    timer = millis();
    g_l_input = 0.3 * (1000 * (g_encoder_l.read() - s_l_last_read)) / (dt * ENCODER_COUNTS_PER_METER) + (0.7 * g_l_input);
    g_r_input = 0.3 * (1000 * (g_encoder_r.read() - s_r_last_read)) / (dt * ENCODER_COUNTS_PER_METER) + (0.7 * g_r_input);
  
    Serial.print('s');
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_l_input);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_r_input);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);

    s_l_last_read = g_encoder_l.read();
    s_r_last_read = g_encoder_r.read();
    return true;
  }
  return false;
}

bool IMU_update() {
  static uint32_t timer = 0;
  if (uint32_t(millis()-timer) > PERIOD_IMU_MS){
    timer = millis();
#ifdef DEBUG_TEST_MSGS
    Serial.print(MSG_IMU);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print("415;1345;-16154;608;-896;-533;-2425;2728;8369");
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);
    return true;
#endif
    
    g_imu.read();
    g_mag.read();
    //g_imu.a.x, g_imu.a.y, g_imu.a.z,
    //g_imu.g.x, g_imu.g.y, g_imu.g.z
    Serial.print(MSG_IMU);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_imu.a.x);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_imu.a.y);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_imu.a.z);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_imu.g.x);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_imu.g.y);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_imu.g.z);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_mag.m.x);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_mag.m.y);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(g_mag.m.z);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);
    return true;
  }
  return false;
}
/*
bool lidar_update() {
  static uint32_t timer = 0;
  unsigned int Distance = 0;
  unsigned int Amplitude = 0;
  if (uint32_t(millis()-timer) > PERIOD_LIDAR_MS){
    timer = millis();
#ifdef DEBUG_TEST_MSGS
//    Serial.print(MSG_LIDAR_READING);
//    Serial.print(MSG_FIELD_SEPARATOR);
//    Serial.print(MSG_TIMESTAMP);
//    Serial.print(MSG_FIELD_SEPARATOR);
//    Serial.print("415;105;");
//    Serial.print(MSG_FIELD_SEPARATOR);
//    Serial.print(MSG_END_BYTE);
    //return true;
#endif

    char result = g_leddar.getDetections();
    if (result >= 0) {
      // Show the first detection only
      Serial.print(MSG_LIDAR_READING);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(MSG_TIMESTAMP);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(g_leddar.Detections[0].Distance);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(g_leddar.Detections[0].Amplitude);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(MSG_END_BYTE);
      return true;
    }
    else {
      //Serial.print("Error: "); Serial.println((int)result); 
      //lcd.setCursor(0,0);
      //mySerial.print("\nError: "); mySerial.print((int)result); mySerial.print("        ");
      //lcd.setCursor(0,1);
      //mySerial.print("\nNo LeddarOne Found");
      //return false;
      Serial.print(MSG_LIDAR_READING);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(0);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(0);
      Serial.print(MSG_FIELD_SEPARATOR);
      Serial.print(MSG_END_BYTE);
    }
  }
  return false;
}
*/

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char in_char = (char)Serial.read();
    // add it to the input_string:
    g_input_string += in_char;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (in_char == '\n') {
      g_string_complete = true;
    }
    else if (g_input_string.length() >= SERIAL_IN_BUFF_SZ) {
      g_input_string = "";
    }
  }
}

void loop() {
  uint8_t index;
 
  encoder_update();
  speed_update ();
  IMU_update();
  //lidar_update();

  g_l_PID.Compute();
   g_motor_l_forward = (g_l_output >= 0);
  g_r_PID.Compute();
  g_motor_r_forward = (g_r_output >= 0);

  //g_motor_l.set(g_l_output);
  //g_motor_r.set(g_r_output);
  g_motor_l.set_dz(g_l_output, MOTOR_PWM_DEAD_ZONE);
  g_motor_r.set_dz(g_r_output, MOTOR_PWM_DEAD_ZONE);

  // Get incomming messages
  if (g_string_complete) {
    
    if (g_input_string[0] == MSG_VELOCITY){
      index = g_input_string.indexOf('MSG_FIELD_SEPARATOR')+1;
      g_l_setpoint = g_input_string.substring(index).toFloat();
      index = g_input_string.indexOf('MSG_FIELD_SEPARATOR',index)+1;
      g_r_setpoint = g_input_string.substring(index).toFloat();
     
      if (g_l_setpoint > -MOTOR_MINIMUM_SPEED && g_l_setpoint < MOTOR_MINIMUM_SPEED) {
        g_l_PID.SetOutputLimits(0, 0.1);
      }
      else {
        g_l_PID.SetOutputLimits(-255, 255);
      }
      if (g_r_setpoint > -MOTOR_MINIMUM_SPEED && g_r_setpoint < MOTOR_MINIMUM_SPEED) {
        g_r_PID.SetOutputLimits(0, 0.1);
      }
      else {
        g_r_PID.SetOutputLimits(-255, 255);
      }
      
    }
    else if (g_input_string[0] == MSG_LIDAR_ANGLE) {
      Serial.println(g_input_string);
     Serial.println( index = g_input_string.indexOf(MSG_FIELD_SEPARATOR)+1);
      g_leddar_servo.write(g_input_string.substring(index).toInt());
      Serial.println(g_input_string.substring(index).toInt());
    }
    
    // clear input string:
    g_input_string = "";
    g_string_complete = false;
  }
  
}

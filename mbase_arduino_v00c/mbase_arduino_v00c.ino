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
#define PIN_BAT_VOLTAGE   A0
#define PIN_BAT_STATE     A6
#define PIN_IMU_VIN       A2
#define PIN_IMU_GND       A3

// Timing and update rates
#define PERIOD_ENCODER_MS         100
#define PERIOD_ENCODER_PULSE_MS   10    // 15 works fine
#define PERIOD_IMU_MS             20    // IMU update period
#define PERIOD_PWR_MON_MS         10000 // Power monitor update period
#define MOTOR_PWM_DEAD_ZONE       100
// Movement threshold
#define MOTOR_MINIMUM_SPEED       0.01
#define SPEED_COMMAND_TIMEOUT_MS  500
#define ENCODER_COUNTS_PER_METER  181.00


// Serial protocol flags
#define MSG_END_BYTE        '\n'
#define MSG_FIELD_SEPARATOR ';'
#define MSG_ODOMETRY        'o'
#define MSG_IMU             'i'
#define MSG_LIDAR_READING   'l'
#define MSG_LIDAR_ANGLE     'a'
#define MSG_VELOCITY        'v'
#define MSG_PWR_MON         'p'

#define MSG_TIMESTAMP       millis()
#define SERIAL_IN_BUFF_SZ   100

#define SERIAL_BAUD         115200


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

PID g_l_PID(&g_l_input, &g_l_output, &g_l_setpoint, 700,300,50, DIRECT);
PID g_r_PID(&g_r_input, &g_r_output, &g_r_setpoint, 700,300,50, DIRECT);
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
  Serial.println("Init OK!");
}

void setup() {
  pinMode(PIN_IMU_VIN, OUTPUT);
  pinMode(PIN_IMU_GND, OUTPUT);
  digitalWrite(PIN_IMU_VIN, HIGH);
  digitalWrite(PIN_IMU_GND, LOW);
  delay(100);
  Serial.begin(SERIAL_BAUD);
  i2c_init();
  g_l_PID.SetOutputLimits(-255, 255);
  g_r_PID.SetOutputLimits(-255, 255);
  g_l_PID.SetSampleTime(50);
  g_r_PID.SetSampleTime(50);
  g_l_PID.SetMode(AUTOMATIC);
  g_r_PID.SetMode(AUTOMATIC);
  g_leddar_servo.attach(PIN_LEDDAR_SERVO);
  pinMode(PIN_BAT_STATE, INPUT_PULLUP);
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

bool power_monitor_update () {
  static uint32_t timer = 0;
  static uint32_t s_last_time;
  static int32_t s_l_last_read, s_r_last_read;
  if (uint32_t(millis()-timer) > PERIOD_PWR_MON_MS){
    timer = millis();

#ifdef DEBUG_TEST_MSGS
    Serial.print(MSG_PWR_MON);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print("1;200");
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);
    return true;
#endif
    Serial.print(MSG_PWR_MON);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_TIMESTAMP);
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(digitalRead(PIN_BAT_STATE)); // will be 0 if power boost indicates low battery
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(analogRead(PIN_BAT_VOLTAGE));  // Analog read of the battery level (1023 * Vbat / 5.0V)
    Serial.print(MSG_FIELD_SEPARATOR);
    Serial.print(MSG_END_BYTE);
    return true;
  }
  return false;
}


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
  static uint32_t s_speed_command_timeout;
  uint8_t index;
  int angle;
  encoder_update();
  speed_update ();
  IMU_update();
  power_monitor_update ();

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
    switch (g_input_string[0]) {
      case MSG_VELOCITY:
        s_speed_command_timeout = millis();
        index = g_input_string.indexOf(MSG_FIELD_SEPARATOR)+1;
        g_l_setpoint = g_input_string.substring(index).toFloat();
        index = g_input_string.indexOf(MSG_FIELD_SEPARATOR,index)+1;
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
      break;
      case MSG_LIDAR_ANGLE:
        index = g_input_string.indexOf(MSG_FIELD_SEPARATOR)+1;
        angle = g_input_string.substring(index).toInt();
        if(angle <= 90 && angle >= -90) {
          g_leddar_servo.write(g_input_string.substring(index).toInt() + 90);
        }
      break;
    }
    // Stop  robot if speed command is not received for a period of SPEED_COMMAND_TIMEOUT_MS
    if (millis() - s_speed_command_timeout > SPEED_COMMAND_TIMEOUT_MS) {
      g_l_PID.SetOutputLimits(0, 0.1);
      g_r_PID.SetOutputLimits(0, 0.1);
    }
    
    // clear input string:
    g_input_string = "";
    g_string_complete = false;
  }
  
}

#include <Wire.h>
//#include <SPI.h>
#include <Servo.h>

#include "define.h"
#include "RC.h"
#include "IMU.h"
#include "POSE.h"
#include "cailibrate.h"
#include "Serial.h"
#include "output.h"


bool terminal_mode = 0;
bool rotor_enable = 1;

unsigned long channal_read_timer;
unsigned long ch1_timer, ch2_timer, ch3_timer, ch4_timer, ch5_timer, ch6_timer;

boolean ch1_s, ch2_s, ch3_s, ch4_s, ch5_s, ch6_s;

void setup() {

  Serial.begin(115200);
  //Serial.setTimeout(5);
  Wire.begin();
  TWBR = 12;

  //motor setup
  DDRD |= B11111100 ;
  
  setup_mpu_6050_registers();
  Gyro_cal();
  angle_read();
  angle_yaw = 0;

  //ISR SETUP
  PCICR |= (1 << PCIE0);
  DDRB = B00000000;
  PCMSK0 = B11111111;

  esc_output_timer = micros();
  ARM_TIMER = millis();

}



void loop() {
  STABLE();

}

ISR(PCINT0_vect) {
  channal_read_timer = micros();
  //ch1
  if (PINB & B00100000) {
    if (ch1_s == 0) {
      ch1_s = 1;
      ch1_timer = channal_read_timer;
    }
  } else if (ch1_s == 1) {
    ch1_s = 0;
    CH[0] = channal_read_timer - ch1_timer;
  }
  //ch2
  if (PINB & B00010000) {
    if (ch2_s == 0) {
      ch2_s = 1;
      ch2_timer = channal_read_timer;
    }
  } else if (ch2_s == 1) {
    ch2_s = 0;
    CH[1] = channal_read_timer - ch2_timer;
  }
  //ch3
  if (PINB & B00001000) {
    if (ch3_s == 0) {
      ch3_s = 1;
      ch3_timer = channal_read_timer;
    }
  } else if (ch3_s == 1) {
    ch3_s = 0;
    CH[2] = channal_read_timer - ch3_timer;
  }
  //ch4
  if (PINB & B00000100) {
    if (ch4_s == 0) {
      ch4_s = 1;
      ch4_timer = channal_read_timer;
    }
  } else if (ch4_s == 1) {
    ch4_s = 0;
    CH[3] = channal_read_timer - ch4_timer;
  }
  //ch5
  if (PINB & B00000010) {
    if (ch5_s == 0) {
      ch5_s = 1;
      ch5_timer = channal_read_timer;
    }
  } else if (ch5_s == 1) {
    ch5_s = 0;
    CH[4] = channal_read_timer - ch5_timer;
  }
  //ch6
    if (PINB & B00000001) {
    if (ch6_s == 0) {
      ch6_s = 1;
      ch6_timer = channal_read_timer;
    }
  } else if (ch6_s == 1) {
    ch6_s = 0;
    CH[5] = channal_read_timer - ch6_timer;
  }
}

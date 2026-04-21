#ifndef PINOUT_H
#define PINOUT_H
// Pinzuordnung für Boreas
const int PIN_LED_DEB = 0;
const int PIN_TEMP_START = 15;

// Motor links
const int PIN_MOTL_B = 12;
const int PIN_MOTL_F = 13;
const int PIN_MOTL_PWM = 18;
const int PIN_MOTL_DIAG = 25;
const int PIN_ENC_LA = 1;
const int PIN_ENC_LB = 2;

// Motor rechts
const int PIN_MOTR_F = 20;
const int PIN_MOTR_B = 21;
const int PIN_MOTR_PWM = 19;
const int PIN_MOTR_DIAG = 22;
const int PIN_ENC_RA = 23;
const int PIN_ENC_RB = 24;

// Bluetooth UART, Serial1 Default Pins
const int PIN_BLUE_TX = 14;
const int PIN_BLUE_RX = 15;

// I2C Interface, Alternative1 Pins
const int PIN_GYRO_SDA = 16;
const int PIN_GYRO_SCK = 17;

// Impeller
const int PIN_IMP = 5;

// IR Sensoren
const int PIN_IR_L = 35;
const int PIN_SEN_L = 36;
const int PIN_IR_LM = 32;
const int PIN_SEN_LM = 33;
const int PIN_IR_M = 30;
const int PIN_SEN_M = 31;
const int PIN_IR_RM = 28;
const int PIN_SEN_RM = 29;
const int PIN_IR_R = 26;
const int PIN_SEN_R = 27;

const int PIN_AKKU = 37;

// Debug UART Serial2, Alternative1 Pins
const int PIN_DEB_TX = 38;
const int PIN_DEB_RX = 39;

#endif
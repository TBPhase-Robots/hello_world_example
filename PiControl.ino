#include <Wire.h>

#define I2C_ADDR  8

//  Defines motor directions
#define FWD LOW
#define REV HIGH

//  Defines motor pins
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device
// Note that the Arduino is limited to
// a buffer of 32 bytes!
#pragma pack(1)
typedef struct i2c_status {
  float x;                  // 4 bytes
  float y;                  // 4 bytes
  float theta;              // 4 bytes
  int8_t status;            // 1 byte
} i2c_status_t;
#pragma pack()

//  Data sent to and from the M5Stack
i2c_status_t i2c_status_tx;
volatile i2c_status_t i2c_status_rx;

//  Drives the left motor
//  Positive velocity is forward, negative reverse
void setLeftMotor(int velocity) {
  if (velocity >= 0) {
    digitalWrite(L_DIR_PIN, FWD);
  }
  else {
    digitalWrite(L_DIR_PIN, REV);
  }

  analogWrite(L_PWM_PIN, abs(velocity));
}

//  Drives the right motor
//  Positive velocity is forward, negative reverse
void setRightMotor(int velocity) {
  if (velocity >= 0) {
    digitalWrite(R_DIR_PIN, FWD);
  }
  else {
    digitalWrite(R_DIR_PIN, REV);
  }

  analogWrite(R_PWM_PIN, abs(velocity));
}

void setup() {
  //  Sets up motor output pins
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  //  Stops both motors
  setLeftMotor(0);
  setRightMotor(0);

  // Serial for debugging.
  Serial.begin(9600);
  Serial.println("***RESTART***");
  delay(1000);

  // Clear out i2c data structs
  memset( (void*)&i2c_status_tx, 0, sizeof( i2c_status_tx ) );
  memset( (void*)&i2c_status_rx, 0, sizeof( i2c_status_rx ) );

  // Begin I2C as a slave device.
  Wire.begin( I2C_ADDR );
  Wire.onRequest( i2c_sendStatus );
  Wire.onReceive( i2c_recvStatus );
}

void loop() {
  //  Do nothing in loop
  delay(100);
}

// When the Core2 calls an i2c request, this function
// is executed.  Sends robot status to Core2.
// Not currently used
void i2c_sendStatus() {

  // Populate our current status
  i2c_status_tx.x = 123.456;
  i2c_status_tx.y = 789.1011;
  i2c_status_tx.theta = 12.13;
  i2c_status_tx.status--; // debugging

  // Send up
  Wire.write( (byte*)&i2c_status_tx, sizeof( i2c_status_tx ) );
}

// When the Core2 calls and i2c write, the robot
// will call this function to receive the data down.
void i2c_recvStatus(int len ) {
  //  Read the i2c status sent by the Core2
  Wire.readBytes( (byte*)&i2c_status_rx, sizeof( i2c_status_rx ) );

  //  Set both motors to run at the speed of the status x value
  setLeftMotor(i2c_status_rx.x);
  setRightMotor(i2c_status_rx.y);
}


void printRXStatus() {
  Serial.println(i2c_status_rx.x);
  Serial.println(i2c_status_rx.y);
  Serial.println(i2c_status_rx.theta);
  Serial.println(i2c_status_rx.status);
}

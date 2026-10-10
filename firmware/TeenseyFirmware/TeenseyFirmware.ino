#include <Arduino.h>
#include <SPI.h>
#include "MT6701.h"
#include <AccelStepper.h>
#include <Servo.h>


//serial comms
uint8_t StartByte = 0xAA;        
int32_t target_steps[7]; 
int32_t encoder_values[7]; 
uint8_t incomingBytes[30];
int byteCount = 0;
unsigned long last_packet_time = 0; //for storing communication time from the PC
int COMS_TIMEOUT = 1000;

//joint gear ratios
const float Gear_Ratios[7] ={3.0, 48.0, 10.0,37.0,3.0,3.0,1.0};
int32_t motor_commands[7] = {0.0}; 
const float steps_per_degree = 4.4; //at 8xmicrostepping 
const float rads_to_degrees = 57.296; //conversion ratio

//loop states
enum ArmState { HOMING, RUNNING, FAULT };
//Servo Defines

#define ServoPin 23
//Stepper Defines
#define J1_Step 0
#define J1_Dir  1
#define J2_Step 2
#define J2_Dir  3
#define J3_Step 4
#define J3_Dir  5
#define J4_Step 6
#define J4_Dir  7
#define J5_Step 24
#define J5_Dir  25
#define J6_Step 26
#define J6_Dir  27

AccelStepper stepper[6] = {
  AccelStepper(AccelStepper::DRIVER, J1_Step, J1_Dir),
  AccelStepper(AccelStepper::DRIVER, J2_Step, J2_Dir),
  AccelStepper(AccelStepper::DRIVER, J3_Step, J3_Dir),
  AccelStepper(AccelStepper::DRIVER, J4_Step, J4_Dir),
  AccelStepper(AccelStepper::DRIVER, J5_Step, J5_Dir),
  AccelStepper(AccelStepper::DRIVER, J6_Step, J6_Dir)
}; //Set up steppers using AccelStepper library

const int stepsPerRevolution = 200; 

//set up max speed and accel Values. TODO calculate realistic values
const float maxSpeed[6] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
const float maxAcceleration[6] = {50.0,50.0,50.0,50.0,50.0,50.0};
//Encoder Defines
#define SlavePin 10
#define MISO 12

// 6 individual CS pins, one per encoder/joint //SCK and MISO are handled in background with the SPI.h package
const int CS_J1 = 22;
const int CS_J2 = 21;
const int CS_J3 = 20; 
const int CS_J4 = 19;
const int CS_J5 = 18;
const int CS_J6 = 17;

const int CS_PINS[6] = {CS_J1, CS_J2, CS_J3, CS_J4, CS_J5, CS_J6};

float encoder_counts[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0};
float derivative_value[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};00000 
float previous_error[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float control_output[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const int tolerance = 0.5;

double encoder_calibrated_offset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//PID Gains
float Kp[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};  // tune per joint
float Ki[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Kd[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
int dt = 1; //delta t
float error[6] = {0, 0, 0, 0, 0, 0};  // error per joint
bool position_reached[6] = {false, false, false, false, false, false};

//6 limit switch pins
const int LS_J1 = 16;
const int LS_J2 = 15;
const int LS_J3 = 41; 
const int LS_J4 = 40;
const int LS_J5 = 39;
const int LS_J6 = 37;
const int LS_PINS[6] = {LS_J1, LS_J2, LS_J3, LS_J4, LS_J5, LS_J6};
const int LS_angle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Limit switch mounting location


AccelStepper stepper1(AccelStepper::DRIVE, J1_Step, J1_Dir);


IntervalTimer controlTimer;


//for homing sequence variable
bool homing_test_complete;
int homing_calibrations_run = 0;

//checksum error count
int checksum_error_count =0;

//control loop to run the control
void controlLoop() {
    // runs every 1ms exactly — read encoders, run PID, send steps
    read_encoders();
    PID_loop();
    move_motors();
}


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

  //stepper motor setup
  for (int i =0; i<6; i++){
    pinMode(Stepper_Step_Pins[i], OUTPUT);
    pinMode(Stepper_Dir_Pins[i], OUTPUT);
    digitalWrite(Stepper_Dir_Pins[i], LOW); //Set initial Direction
  }

    //set stepper speed and acceleration max
    for (int i=0; i<6; i++){
      stepper[i].setMaxSpeed(maxSpeed[i]);
      stepper[i].setAcceleration(maxAcceleration[i]);
    }
    //Encoder Setup
    for (int i = 0; i < 6; i++) {
      pinMode(CS_PINS[i], OUTPUT);
      digitalWrite(CS_PINS[i], HIGH);  // all deselected at boot
    }

    //servo motor setup
    myServo.attach(ServoPin);
    SPI.begin();

    arm_state = HOMING;

    controlTimer.begin(controlLoop, 1000);  // 1000 microseconds = 1kHz
}

void loop() {
  
  switch (arm_state){
    case HOMING: {
        homing_test_complete = homing_sequence();
        if (homing_test_complete) {
            arm_state = RUNNING;
            homing_calibrations_run = 0;
        } else {
            homing_calibrations_run += 1;
            if (homing_calibrations_run >= 3) {
                Serial.println("WARNING: homing failed 3x");
                arm_state = FAULT;          // fall into shutdown next iteration
            }
        }
        break;
    }

    case RUNNING:{
      // handle serial communication with PC
        if(millis() - last_packet_time > COMS_TIMEOUT){
          Serial.println("WARNING: COMS LOST. Restart Process");
          arm_state = FAULT;
          break;
      }
      while(Serial.available() && byteCount<30){
        incomingBytes[byteCount++] = Serial.read();

        if (byteCount == 1 && incomingBytes[0] != StartByte){ //TODO this might hang here, add error redundancy code here
          byteCount =0;
        }
      }
        
      if (byteCount >= 30){
        byteCount =0;

        if(incomingBytes[29] != computeChecksum(incomingBytes)){
          checksum_error_count += 1;
          continue;//exit and ignore
        }
        checksum_error_count = 0;

        for (int i=0; i < 7; i++){
          memcpy(&target_steps[i], &incomingBytes[1 + i*4], 4);
          target_steps[i] = target_steps[i]*rads_to_degrees; //converts radians to degrees
          
        }
        last_packet_time = millis();
      }
      
      if (checksum_error_count >=10){ //If ten packets in a row are lost, throw a FAULT
        Serial.println("WARNING: Motors Shut Down. Restart Process");
        arm_state = FAULT;
      }
      read_encoders();
      write_encoder_Values();
      PID_loop(encoder_counts);
      move_motors();
      for(int i = 0; i<6; i++){
        if(digitalRead(LS_PINS[i])){
          arm_state = FAULT;
          break;
        }        
      }
      myServo.write(encoder_counts[6]);
      break;
      
    }
    case FAULT:
      for (int i = 0; i < 6; i++) {
          stepper[i].setSpeed(0);              // stop commanding steps
          digitalWrite(EN_PINS[i], HIGH);      // <-- POLARITY: see note below
      }

      static bool error_triggered = false;
      if (!error_triggered) {                           // print once, not every frame
          Serial.println("WARNING: Motors Shut Down. Restart Process");
          error_triggered = true;
      }
      return;


}

//checksum function for packets to and from PC
uint8_t computeChecksum(uint8_t buffer[30]){
  uint8_t checksum_value = 0;
  for (int i = 0; i < 29; i++){
    checksum_value += buffer[i];
  }
  return checksum_value;
}

//Function to loop and read encoders
void read_encoders() {

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2)); // Start SPI transaction
    for (int i = 0; i < 6; i++) {
        uint8_t buffer[3];
        uint16_t encoder_angle = 0;
        digitalWrite(CS_PINS[i], LOW);   // select encoder i
        SPI.transfer(buffer, 3);
        // SPI transfer to read angle
        digitalWrite(CS_PINS[i], HIGH);  // deselect
        
        if(!CRCCheck(buffer)){
          continue;
        }

        //combine the 8 bit bytes from buffer into one 24 bit encoder paket
        encoder_angle = (buffer[0]<<6)|(buffer[1]>>2);
        encoder_counts[i] = ((encoder_angle*360)/16384)-encoder_calibrated_offset[i];

    }
    SPI.endTransaction(); // End SPI transaction
}

void move_motors(){

  for(int i=0; i < 6; i++){

    if(position_reached[i]){
      continue
    }
    stepper[i].setSpeed(control_output[i]);
    stepper[i].runSpeed();
  }

}

bool CRCCheck(uint8_t buffer[3]){
  //create empty variables
  uint32_t encoder_data = 0;
  uint8_t crcCheck = 0;
  uint8_t checkResult =0;

  //store polynomial
  //extract encoder and magnet data
  encoder_data = (buffer[0]<<10)|(buffer[1]<<2)|(buffer[2]>>6);
  //extract crc data
  crcCheck = buffer[2] & ((1 << 6) - 1);
  //divide by crc
    for (int i = 17; i >= 0; i--) {          // 18 data bits, MSB first
        uint8_t data_bit = (encoder_data >> i) & 1;
        uint8_t top_bit  = (checkResult >> 5) & 1;   // bit 5 = top of a 6-bit register
        checkResult = (checkResult << 1) & 0x3F;             // shift left, keep to 6 bits
        if (top_bit ^ data_bit) {
            checkResult ^= 0x03;                     // XOR low 6 bits of the polynomial
        }
    }
  //compare
  return checkResult == crcCheck;

}

void PID_loop(float encoder_counts[6]){

  for (int i = 0; i<6; i++){
  previous_error[i] = error[i];
  error[i] = target_steps[i] - encoder_counts[i];

  if(fabs(error[i])>180){ //used for when joints do full loop, example, going from 355 to 05 degrees

    if (error[i] < 0){ //for CW rollover
      target_steps[i] = 360 + target_steps[i];
    }

    if (error[i] > 0){ // For CCW rollover
      target_steps[i] = 360 - target_steps[i];
    }
    
    error[i] = target_steps[i] - encoder_counts[i];
  }

  derivative_value[i] = (error[i]-previous_error[i]) /dt;

  if (fabs(error[i]) <= 5){
    integral_value[i] += error[i]* dt;
  }

  if (fabs(error[i]) <= tolerance){
    integral_value[i] = 0;
    position_reached[i] = true;
  }

  else{
    position_reached[i] = false;
  }

  control_output[i] = Kp[i]*error[i] + integral_value[i]*Ki[i] + Kd[i]*derivative_value[i];
}

void write_encoder_values(){
  uint8_t buffer[30];
  buffer[0] = 0x00;
  for (size_t i = 0; i<7; i++)
  {
      float  position = encoder_counts[i]/rads_to_degrees;
      
      //convert from differential mechanism to pitch and roll for the wrist mechanism
      if(i==4){ 
        position = (encoder_counts[i]+encoder_counts[i+1])/(rads_to_degrees*2);
      }

      if(i==5){
        position = (encoder_counts[i-1]-encoder_counts[i])/(rads_to_degrees*2);
      }
      memcpy(&buffer[1+i*4], &position, 4);
  }
  buffer[29] = computeChecksum(buffer);
  Serial.write(buffer, 30);
}

bool homing_sequence(){

  int encoder_reading_check = 0;
  bool homing_sequence_complete = false;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2)); // Start SPI transaction
  for(int i=0; i < 6; i++){
    
    stepper[i].setSpeed(5);

    while(!digitalRead(LS_PINS[i])){

      stepper[i].runSpeed();
    }

    stepper[i].setCurrentPosition(0);


    uint8_t buffer[3];
    uint16_t encoder_angle = 0;
    digitalWrite(CS_PINS[i], LOW);   // select encoder i
    SPI.transfer(buffer, 3);
    // SPI transfer to read angle
    digitalWrite(CS_PINS[i], HIGH);  // deselect
      
    if(!CRCCheck(buffer)){
      continue;
    }
    //combine the 8 bit bytes from buffer into one 24 bit encoder paket
    encoder_angle = (buffer[0]<<6)|(buffer[1]>>2);
    encoder_calibrated_offset[i] = ((encoder_angle*360)/16384)-(LS_angle[i]);
    
    if(encoder_angle[i]<= 360 && encoder_angle[i] >= 0){
      encoder_reading_check+=1;
    }
    continue;
  
  }
  SPI.endTransaction(); // End SPI transaction


  if(encoder_reading_check == 6){
    homing_sequence_complete = true;
  }

  return homing_sequence_complete;

}


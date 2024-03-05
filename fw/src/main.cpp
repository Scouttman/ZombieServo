#include <Arduino.h>
#include <SimpleFOC.h>
#include "drivers/stspin32g4/STSPIN32G4.h"
// #include "encoders/ma730/MA730.h"
#include "encoders/ma730/MagneticSensorMA730.h"

#define LED_WHITE PB_10 
#define LED_RED PC_13
#define MAG_CS PA4
#define MOIS_PIN PB5
#define MISO_PIN PB4
#define SCLK_PIN PB3
#define SSI_SSCK_FREQUENCY  10000       //SSI SSCK Clock frequency in Hz #TODO increase back to 100k

STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(1); // for drive module
HallSensor sensor = HallSensor(PC0, PC2, PC3, 1);
MagneticSensorMA730 output_encoder = MagneticSensorMA730(MAG_CS);
// interrupt routine intialisation
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// External digital encoder
SPIClass SPI_1(MOIS_PIN, MISO_PIN, SCLK_PIN);
PIDController PID_ang{1000.0,0.0,0.0,0,5000}; // PID controller for the output angle

// angle set point variable
float target_angle = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }

int counter = 0;

void setup() {
  Serial.begin();
  Serial.println("Start");
  command.add('M',doMotor,"motor");
  // add target command T
  command.add('T', doTarget, "target angle");

  // setup timer 6 to count micro seconds as the micro second timer inside stduino is slow
  __HAL_RCC_TIM6_CLK_ENABLE();
  TIM6->PSC = HAL_RCC_GetPCLK1Freq()/1000000 - 1;
  TIM6->CR1 = TIM_CR1_CEN;

  /* Initialize all configured peripherals */
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // initialise encoder hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);
  output_encoder.init(&SPI_1);

  // driver
  driver.voltage_power_supply = 17.0f;
  driver.voltage_limit = 17.0f;
  driver.init();
  motor.voltage_limit = driver.voltage_limit / 2.0f;
  motor.controller = MotionControlType::velocity;
  // set the torque control type
  motor.phase_resistance = 12.5; // 12.5 Ohms
  // set motion control loop to be used
  // motor.controller = MotionControlType::torque;
  motor.linkDriver(&driver);
  motor.init();

  motor.PID_velocity.P = 0.001f;
  motor.PID_velocity.I = 0.0f;
  motor.PID_velocity.D = 0;

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
  // driver.voltage_limit = 6;

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
}

void loop() {
    // main FOC algorithm function
  motor.loopFOC();
  
  float vel_set = PID_ang(output_encoder.getAngle()-target_angle);
  motor.move(vel_set);
  if(driver.isFault()){
    digitalWrite(LED_WHITE, HIGH);
    digitalWrite(LED_RED, HIGH);
  }else{
    digitalWrite(LED_WHITE, LOW);
    digitalWrite(LED_RED, LOW);
  }
  // real-time monitoring calls
  motor.monitor();
  // real-time commander calls
  command.run();
  if(counter++%10000 == 0){
    Serial.print("Ang:");
    Serial.println(output_encoder.getAngle()); 

  }
  output_encoder.update();
}
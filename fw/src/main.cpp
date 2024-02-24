#include <Arduino.h>
#include <SimpleFOC.h>
#include "drivers/stspin32g4/STSPIN32G4.h"

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
// interrupt routine intialisation
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

float get_ang();
SPIClass SPI_1(MOIS_PIN, MISO_PIN, SCLK_PIN);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }
int counter = 0;


void setup() {
  Serial.begin();
  Serial.println("Start");
  command.add('M',doMotor,"motor");


  // setup timer 6 to count micro seconds as the micro second timer inside stduino is slow
  __HAL_RCC_TIM6_CLK_ENABLE();
  TIM6->PSC = HAL_RCC_GetPCLK1Freq()/1000000 - 1;
  TIM6->CR1 = TIM_CR1_CEN;

  /* Initialize all configured peripherals */
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  SPI_1.begin(MAG_CS);
  SPI_1.beginTransaction(SPISettings(SSI_SSCK_FREQUENCY, MSBFIRST, SPI_MODE0));

  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
   
  // initialise encoder hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);

  // driver
   driver.voltage_power_supply = 17.0f;
  driver.voltage_limit = 17.0f;
  driver.init();
  motor.voltage_limit = driver.voltage_limit / 2.0f;
  motor.controller = MotionControlType::velocity;
  // set the torque control type
  motor.phase_resistance = 12.5; // 12.5 Ohms
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  motor.linkDriver(&driver);
  motor.init();

  motor.PID_velocity.P = 0.005f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
  // driver.voltage_limit = 6;

  Serial.println(F("Motor ready."));
}

void loop() {
    // main FOC algorithm function
  motor.loopFOC();
  motor.move(50.0f); // Try to drive it as fast as possible
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
  if(counter++%1000 == 0){
    // Serial.println(get_ang());
    // Serial.println((sensor.getVelocity()/_2PI)*60);
  }
}


float get_ang(){
  byte response1 = SPI_1.transfer(MAG_CS, 0x00, SPI_CONTINUE);
  //transfer 0x00 to the device on pin 10, store byte received in response2, deselect the chip
  byte response2 = SPI_1.transfer(MAG_CS, 0x00);
  int ang_raw = (response1<<8) + response2;
  return (ang_raw*180.0)/65536.0;
}
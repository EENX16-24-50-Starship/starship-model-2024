/*
* This is the main file for the Starship model. 
* It contains the setup and loop functions, as well as the functions for initializing and reading the sensors.
* 
* By Emlzdev (Emil Reinfeldt)
*/

// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Dps3xx.h>
//#include "I2Cdev.h"
#include "RF24.h"
#include <nRF24L01.h>
#include "GlobalDecRocket.h"
#include "RadioTransceiverMaster.h"

// Change debug mode | COMMENT OUT WHEN NO COMPUTER CONNECTED
#define DEBUG



// =============================================================================================
//  Constants
// =============================================================================================

// Sensor adresses for I2C
#define IMU_ADR 0x68 //b1101000
//#define PRESSURE_SENSOR_ADR 0x77 // Default and does not need to be given

// Baudrate for serial communication to terminal on computer
#define BAUDRATE 250000 

// LED pin on microcontroller
#define LED_PIN 13

// Timeout to wait before skipping a task
#define TIMEOUT_DURATION 15000000 // 15 seconds

// Delay between pressure sensor readings
#define PS_DELAY 240 // 240 milliseconds

// ====== Radio Configuration ======
// Define the pins used for the nRF24L01 transceiver module (CE, CSN)
#define CE_PIN 7
#define CSN_PIN 8
// Define transmit power level | RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define RF24_PA_LEVEL RF24_PA_MIN
// Define speed of transmission | RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define RF24_SPEED RF24_2MBPS
// What radio channel to use (0-127). The same on all nodes must match exactly.
#define RF24_CHANNEL 124 



// =============================================================================================
//  Variables/Objects
// =============================================================================================

// Pressure sensor object
Dps3xx Dps3xxPressureSensor = Dps3xx();

/*
  * temperature measure rate (value from 0 to 7)
  * 2^temp_mr temperature measurement results per second
  */
int16_t temp_mr = 2;

/*
  * temperature oversampling rate (value from 0 to 7)
  * 2^temp_osr internal temperature measurements per result
  * A higher value increases precision
  */
int16_t temp_osr = 2;
 
/*
  * pressure measure rate (value from 0 to 7)
  * 2^prs_mr pressure measurement results per second
  */
int16_t prs_mr = 2;

/*
  * pressure oversampling rate (value from 0 to 7)
  * 2^prs_osr internal pressure measurements per result
  * A higher value increases precision
  */
int16_t prs_osr = 2;

// IMU object
// TODO: Add IMU object

// IMU sensor data
// TODO: Add IMU sensor data variables

// LED pin state
bool blinkState = false;

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Indicateds if there is new data to be read from the radio
bool newControllerData = false;

// For when to send packets
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 250; // send once per every 250 milliseconds

// Create a Packet to hold the data
PacketData senderData;

// Acknowledge payload to hold the data coming from the rocket
ControlData ackData;



// =============================================================================================
//  Functions
// =============================================================================================

// Initialize pressure sensor
void initDPS310(){

  #ifdef DEBUG
    Serial.println("Initializing Pressuresensor over I2C...");
  #endif
  Dps3xxPressureSensor.begin(Wire);
  #ifdef DEBUG
    Serial.println("Pressuresensor I2C initialized successfully");
  #endif

  /*
   * startMeasureBothCont enables background mode
   * temperature and pressure ar measured automatically
   * High precision and hgh measure rates at the same time are not available.
   * Consult Datasheet (or trial and error) for more information
   */
  int16_t ret = Dps3xxPressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
  
  /*
   * Use one of the commented lines below instead to measure only temperature or pressure
   * int16_t ret = Dps3xxPressureSensor.startMeasureTempCont(temp_mr, temp_osr);
   * int16_t ret = Dps3xxPressureSensor.startMeasurePressureCont(prs_mr, prs_osr);
   */
  #ifdef DEBUG
  if (ret != 0)
  {
    Serial.print("Init Dsp310 FAILED! ret = ");
    Serial.println(ret);
  }
  else
  {
    Serial.println("Init Dsp310 complete!");
  }
  #else
  if (ret != 0)
  {
    // TODO: Initialization failed, send error to computer
  }
  #endif
}

// Initialize IMU
void initIMU(){

  // TODO: Initialize IMU sensor correctly
  
}

// Read IMU data from I2C bus
void readIMU(){

  // TODO: Add funtions to get data, GUNNAR

}

// Read temperature and pressure
void readPS(){

  static unsigned long lastTime = 0; // Keep track of the last time we read the sensor
  unsigned long currentTime = millis(); // Get the current time

  // Only read the sensor if at least 240 milliseconds have passed since the last reading
  if (currentTime - lastTime >= PS_DELAY) {

    uint8_t pressureCount = 20;
    float pressure[pressureCount];
    uint8_t temperatureCount = 20;
    float temperature[temperatureCount];

    /*
    * This function writes the results of continuous measurements to the arrays given as parameters
    * The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
    * After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
    * Note: The Dps3xx cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
    */
    int16_t ret = Dps3xxPressureSensor.getContResults(temperature, temperatureCount, pressure, pressureCount);

    #ifdef DEBUG
    if (ret != 0)
    {
      Serial.println();
      Serial.println();
      Serial.print("FAIL! ret = ");
      Serial.println(ret);
    }
    else
    {
      Serial.println();
      Serial.println();
      Serial.print(temperatureCount);
      Serial.println(" temperature values found: ");
      for (int16_t i = 0; i < temperatureCount; i++)
      {
        Serial.print(temperature[i]);
        Serial.println(" degrees of Celsius");
      }

      Serial.println();
      Serial.print(pressureCount);
      Serial.println(" pressure values found: ");
      for (int16_t i = 0; i < pressureCount; i++)
      {
        Serial.print(pressure[i]);
        Serial.println(" Pascal");
      }
    }
    #else
    // In non-debug mode we can just use the data
    if (ret != 0)
    {
      // TODO: Send error message to computer
    }
    else
    {
      // TODO: Save data in variables to LQR
      Serial.print(temperatureCount);
      for (int16_t i = 0; i < temperatureCount; i++)
      {
        Serial.print(temperature[i]);
      }

      Serial.print(pressureCount);
      for (int16_t i = 0; i < pressureCount; i++)
      {
        Serial.print(pressure[i]);
      }
    }

    #endif
    lastTime = currentTime; // Update the last time we read the sensor
  }
}



// =============================================================================================
//  Main Program
// =============================================================================================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(BAUDRATE); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  #ifdef DEBUG
  Serial.println("==== Starship model initializing... ====");
  Serial.println("");
  Serial.println("Initializing I2C bus...");
  #endif

  // Initialize radio module
  initRadio(radio, RF24_PA_LEVEL, RF24_SPEED, RF24_CHANNEL);

  // Initialize I2C bus
  Wire.begin();

  // Initialize sensors
  //initIMU();
  initDPS310();

  // Configure microcontroller LED for TX/RX status
  pinMode(LED_PIN, OUTPUT);

  #ifdef DEBUG
  Serial.println("Init complete!");
  #endif
}

void loop() {
  //readIMU();
  readPS();

  // Blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  // Send the data to the ground controller via radio
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    if(!transmitData(radio, senderData, ackData, newControllerData, prevMillis)){
      // Connection lost to the ground controller
      // Set flag and try reconnecting in the next loop
      // TODO: Initialize landing script and dearm the rocket
    }
  }

}
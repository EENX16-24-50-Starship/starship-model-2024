// IMU testing script
// ---------------------
#include <Arduino.h>
#include "ICM20600.h"
#include <Wire.h>
#include "AK09918.h"
#include "MadgwickAHRS.h"
#include "Dps310.h"
#include <SimpleFOC.h>

const float ALPHA = 0.35; //0.35;                        // <<<<<<<-----------------------------------------------------Increase this for faster response

//Initialize components----------------------------------------------------------------------------------------------------
ICM20600 icm20600(true);
AK09918_err_type_t err;
AK09918 ak09918;
Dps310 Dps310PressureSensor = Dps310();
Madgwick filter;
LowPassFilter filter_LP_roll = LowPassFilter(ALPHA);
LowPassFilter filter_LP_pitch = LowPassFilter(ALPHA);
LowPassFilter filter_LP_yaw = LowPassFilter(ALPHA);

//Global variables---------------------------------------------------------------------------------------------------------
// Program parameter settings
double declination_shenzhen = 4.95; //4.78;      //From http://www.magnetic-declination.com/
const float sensorRate = 100; //25.00; //[Hz]    // <<<<<<<-----------------------------------------------------Adjust this sensor update freq.

// States
int32_t magn_x, magn_y, magn_z;
int32_t offset_x, offset_y, offset_z;
float roll_filtered, pitch_filtered, yaw_filtered;
float roll_filtered_, pitch_filtered_, yaw_filtered_;

unsigned long start_time, current_time, tPrint;

// IMU calibration offsets
// float x_acc_offset = 0;
// float y_acc_offset = 0;
float z_acc_offset = 0;

// float x_rot_offset = 0;
// float x_rot_offset = 0;
float z_rot_offset = 0;


// Routines----------------------------------------------------------------------------------------------------------------
// Mangetometer calibration (from IMU library)
void calibrate_magn(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;

    ak09918.getData(&magn_x, &magn_y, &magn_z);

    value_x_min = magn_x;
    value_x_max = magn_x;
    value_y_min = magn_y;
    value_y_max = magn_y;
    value_z_min = magn_z;
    value_z_max = magn_z;
    delay(100);

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        ak09918.getData(&magn_x, &magn_y, &magn_z);
        
        if (value_x_min > magn_x){
            value_x_min = magn_x;
        } 
        else if (value_x_max < magn_x){
            value_x_max = magn_x;
        }
        if (value_y_min > magn_y){
            value_y_min = magn_y;
        } 
        else if (value_y_max < magn_y){
            value_y_max = magn_y;
        }
        if (value_z_min > magn_z){
            value_z_min = magn_z;
        } 
        else if (value_z_max < magn_z){
            value_z_max = magn_z;
        }
        
        Serial.print(".");
        delay(100);
    }
    *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
    *offsety = value_y_min + (value_y_max - value_y_min) / 2;
    *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
}

// Calibration of gyro and accelerometer
// -------------------------------------
// Functions by calculate avrage reading over a 10 
// second period while the IMU is level and not mowing
void calibrate_IMU_zAxis(uint32_t timeout, float *z_accOffset, float *z_rotOffset) {
    int32_t value_zAcc_min = 0;
    int32_t value_zAcc_max = 0;
    int32_t value_zRot_min = 0;
    int32_t value_zRot_max = 0;
    uint32_t timeStart = 0;

    // Get initial values
    // Acceleration [mg]
    float zAcc = icm20600.getAccelerationZ();

    // Rotation [dps]
    float zGyro = icm20600.getGyroscopeZ();

    value_zAcc_min = zAcc;
    value_zAcc_max = zAcc;

    value_zRot_min = zGyro;
    value_zRot_max = zGyro;
    delay(100);

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        float zAcc = icm20600.getAccelerationZ();
        float zGyro = icm20600.getGyroscopeZ();

        // Check acceleration min / max
        if (value_zAcc_min > zAcc){
          value_zAcc_min = zAcc;
        } 
        else if (value_zAcc_max < zAcc){
          value_zAcc_max = zAcc;
        }
        
        // Check rot acceleration min / max
        if (value_zRot_min > zGyro){
          value_zRot_min = zGyro;
        } 
        else if (value_zRot_max < zGyro){
          value_zRot_max = zGyro;
        }

        Serial.print(".");
        delay(100);
    }

    // Calculate avrages and assign offsets
    *z_accOffset = value_zAcc_min + (value_zAcc_max - value_zAcc_min) / 2;
    *z_rotOffset = value_zRot_min + (value_zRot_max - value_zRot_min) / 2;
}

void LP_filter(float roll, float pitch, float yaw){
  roll_filtered_ = ALPHA*roll+(1-ALPHA)*roll_filtered;
  pitch_filtered_ = ALPHA*pitch+(1-ALPHA)*pitch_filtered;
  yaw_filtered_ = ALPHA*yaw+(1-ALPHA)*yaw_filtered;
}


// Setup---------------------------------------------------------
void setup() {
  start_time = millis();
  Wire.begin();
  filter.begin(sensorRate);

  Serial.begin(115200);
  
  // Sensor setup
  err = ak09918.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
  icm20600.initialize();

  // Code from library examples
  err = ak09918.isDataReady();
  while (err != AK09918_ERR_OK) {
      Serial.println("Waiting Sensor");
      delay(100);
      err = ak09918.isDataReady();
  }

  // Calibrate magnetic sensor, takes 10 seconds
  Serial.print("Calibrating magnetic sensors for 10 seconds");
  Serial.print('\n');
  calibrate_magn(10000, &offset_x, &offset_y, &offset_z);

  // Calibration of gyroscope (to mitigate drift)
  Serial.print("Calibrating gyro & accelerometer sensors for 5 seconds");
  Serial.print('\n');
  calibrate_IMU_zAxis(5000, &z_acc_offset, &z_rot_offset);
}


// Main loop--------------------------------------------------
void loop() {
  // Init variables
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;

  float roll, pitch, yaw;

  // Fetch IMU data
  // Acceleration [mg]
  xAcc = icm20600.getAccelerationX();
  yAcc = icm20600.getAccelerationY();
  zAcc = icm20600.getAccelerationZ();

  // Rotation [dps]
  xGyro = icm20600.getGyroscopeX();
  yGyro = icm20600.getGyroscopeY();
  zGyro = icm20600.getGyroscopeZ() - z_rot_offset;

  // Fetch MAG data [uT] (micro-tesla)
  ak09918.getData(&magn_x, &magn_y, &magn_z);
  float m_x = float(magn_x - offset_x);
  float m_y = float(magn_y - offset_y);
  float m_z = float(magn_z - offset_z);
 

  // Estimate states with madgwick filter  
  // filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
  xGyro = filter_LP_roll(xGyro);
  yGyro = filter_LP_pitch(yGyro);
  zGyro = filter_LP_yaw(zGyro);

  filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, m_x, m_y, m_z);
  
  roll = filter.getRoll();            //x-axis on IMU 
  pitch = filter.getPitch();          //y-axis on IMU
  yaw = filter.getYaw()-180.0;        //z-axis on IMU

  // LP-filtering of the estimated states
  //LP_filter(roll,pitch,yaw);
  // roll_filtered = filter_LP_roll(roll);
  // pitch_filtered = filter_LP_pitch(pitch);
  // yaw_filtered = filter_LP_yaw(yaw);


  // Print sensor data to serial port
  // --------------------------------
  // Orientation output from madgwick filter
  Serial.print("Madgwick rot. xyz:");
  Serial.print('\t');

  Serial.print(roll_filtered);
  Serial.print('\t');

  Serial.print(pitch_filtered);
  Serial.print('\t');

  Serial.print(yaw_filtered);
  Serial.print('\n');

/*
  // Unfiltered gyro data
  Serial.print("Gyro xyz:");
  Serial.print('\t');

  Serial.print(xGyro);
  Serial.print('\t');

  Serial.print(yGyro);
  Serial.print('\t');

  Serial.print(zGyro);
  Serial.print('\n');
*/
/*
  // Unfiltered acceleration data
  Serial.print("Acc xyz:");
  Serial.print('\t');

  Serial.print(xAcc);
  Serial.print('\t');

  Serial.print(yAcc);
  Serial.print('\t');

  Serial.print(zAcc);
  Serial.print('\n');

    
  // Unfilitered magnetic compass data
  Serial.print("Magn xyz:");
  Serial.print('\t');

  Serial.print(magn_x);
  Serial.print('\t');

  Serial.print(magn_y);
  Serial.print('\t');

  Serial.print(magn_z);
  Serial.print('\n');
*/
}


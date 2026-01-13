#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "ArduHelpers.h"
#include "EigenHelpers.h"
#include "IGRF.h"
#include "FiltAndCal.h"
#include "INS.h"
#include "eigen.h"
#include <Eigen/Geometry>




using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;




#define LSM_CS     32
#define LIS3MDL_CS 31




float lat =  40.7128;
float lon = -74.0060;




Adafruit_LSM6DSOX imu;
Adafruit_LIS3MDL  mag;

sensors_event_t accelEvent;
sensors_event_t gyroEvent;
sensors_event_t tempEvent;
sensors_event_t magEvent;

ProcessingConfigVector accelProcConfig;
ProcessingConfigVector gyroProcConfig;
ProcessingConfigVector magProcConfig;

ProcessVector accelProc;
ProcessVector gyroProc;
ProcessVector magProc;

CompFilt compAHRS;

Vector3f igrf;




void setupIGRF()
{
    igrf << igrf_Bn(lat, lon),
            igrf_Be(lat, lon),
            igrf_Bd(lat, lon);
    
    compAHRS.updateMag(igrf);
}




void setupIMU()
{
    if (!imu.begin_SPI(LSM_CS, &SPI, 10e6))
    {
        Serial.println("Failed to find LSM6DSOX chip");
        while (1) { delay(10); }
    }

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

    accelProc.begin(accelProcConfig);
    gyroProc.begin(gyroProcConfig);
}




void setupMag()
{
    if (!mag.begin_SPI(LIS3MDL_CS, &SPI, 10e6))
    {
        Serial.println("Failed to find LIS3MDL chip");
        while (1) { delay(10); }
    }

    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);

    magProc.begin(magProcConfig);
}




void setup()
{
    Serial.begin(115200);

    forceAllPinsHigh();

    setupIGRF();
    setupIMU();
    setupMag();

    compAHRS.updateTau(0.5);
}




void loop()
{
    unsigned long timestamp = micros();

    imu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    mag.getEvent(&magEvent);

    Vector3f rawAccel;
    rawAccel << -accelEvent.acceleration.x,
                -accelEvent.acceleration.y,
                 accelEvent.acceleration.z;

    Vector3f rawGyro;
    rawGyro << -gyroEvent.gyro.x,
               -gyroEvent.gyro.y,
                gyroEvent.gyro.z;

    Vector3f rawMag;
    rawMag <<  magEvent.magnetic.y,
              -magEvent.magnetic.x,
               magEvent.magnetic.z;
    
    // Vector3f calAccel = accelProc.process(rawAccel);
    // Vector3f calGyro  = gyroProc.process(rawGyro);
    // Vector3f calMag   = magProc.process(rawMag);

    compAHRS.update(rawAccel, rawGyro, rawMag, timestamp);
    Vector3f eulers = compAHRS.get_b_R_ang_n();

    Serial.print(eulers(0));
    Serial.print(',');
    Serial.print(eulers(1));
    Serial.print(',');
    Serial.print(eulers(2));
    Serial.println();

    delay(1);
}

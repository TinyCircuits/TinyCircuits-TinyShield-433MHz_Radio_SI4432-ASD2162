//-------------------------------------------------------------------------------
//  TinyCircuits LSM9DS1 9 Axis TinyShield Example Sketch
//  Last Updated 5 May 2016
//  
//  This demo is intended for the ASD2511 Sensor Board TinyShield with a LSM9DS1
//  9 axis sensor populated. It shows basic use of a modified RTIMULib with the
//  sensor.
//
//  Modified by Ben Rose for TinyCircuits, https://tinycircuits.com
//
//-------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

#ifdef SERIAL_PORT_MONITOR
  #define SERIAL SERIAL_PORT_MONITOR
#else
  #define SERIAL Serial
#endif

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

void setup()
{
    int errcode;
  
    SERIAL.begin(SERIAL_PORT_SPEED);
    while(!SERIAL);
    
    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    SERIAL.print("ArduinoIMU starting using device "); SERIAL.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        SERIAL.print("Failed to init IMU: "); SERIAL.println(errcode);
    }

    if (imu->getCalibrationValid())
        SERIAL.println("Using compass calibration");
    else
        SERIAL.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
}

void loop()
{  
  unsigned long now = millis();
  unsigned long delta;
  
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      SERIAL.print("Sample rate: "); SERIAL.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        SERIAL.println(", gyro bias valid");
      else
        SERIAL.println(", calculating gyro bias - don't move IMU!!");
        
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      RTVector3 accelData=imu->getAccel();
      RTVector3 gyroData=imu->getGyro();
      RTVector3 compassData=imu->getCompass();
      RTVector3 fusionData=fusion.getFusionPose();
      //displayAxis("Accel:", accelData.x(), accelData.y(), accelData.z());        // accel data
      //displayAxis("Gyro:", gyroData.x(), gyroData.y(), gyroData.z());            // gyro data
      //displayAxis("Mag:", compassData.x(), compassData.y(), compassData.z());    // compass data
      displayDegrees("Pose:", fusionData.x(), fusionData.y(), fusionData.z());   // fused output
      SERIAL.println();
    }
  }
}

void displayAxis(const char *label, float x, float y, float z)
{
  SERIAL.print(label);
  SERIAL.print(" x:"); SERIAL.print(x);
  SERIAL.print(" y:"); SERIAL.print(y);
  SERIAL.print(" z:"); SERIAL.print(z);
}

void displayDegrees(const char *label, float x, float y, float z)
{
  SERIAL.print(label);
  SERIAL.print(" x:"); SERIAL.print(x * RTMATH_RAD_TO_DEGREE);
  SERIAL.print(" y:"); SERIAL.print(y * RTMATH_RAD_TO_DEGREE);
  SERIAL.print(" z:"); SERIAL.print(z * RTMATH_RAD_TO_DEGREE);
}
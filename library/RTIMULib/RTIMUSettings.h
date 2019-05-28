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
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTIMUSETTINGS_H
#define _RTIMUSETTINGS_H

#include "RTMath.h"
#include "RTIMULibDefs.h"

class RTIMUSettings
{
public:
    RTIMUSettings();

    //  These are the local variables

    int m_imuType;                                          // type code of imu in use
    unsigned char m_I2CSlaveAddress;                        // I2C slave address of the imu
    int m_pressureType;                                     // type code of pressure sensor in use
    unsigned char m_I2CPressureAddress;                     // I2C slave address of the pressure sensor

    //  IMU-specific vars

#if defined(MPU9150_68) || defined(MPU9150_69)
    //  MPU9150

    int m_MPU9150GyroAccelSampleRate;                       // the sample rate (samples per second) for gyro and accel
    int m_MPU9150CompassSampleRate;                         // same for the compass
    int m_MPU9150GyroAccelLpf;                              // low pass filter code for the gyro and accel
    int m_MPU9150GyroFsr;                                   // FSR code for the gyro
    int m_MPU9150AccelFsr;                                  // FSR code for the accel
#endif

#if defined(LSM9DS1)
    //  LSM9DS1

    int m_LSM9DS1GyroSampleRate;                            // the gyro sample rate
    int m_LSM9DS1GyroBW;                                    // the gyro bandwidth code
    int m_LSM9DS1GyroHpf;                                   // the gyro high pass filter cutoff code
    int m_LSM9DS1GyroFsr;                                   // the gyro full scale range

    int m_LSM9DS1AccelSampleRate;                           // the accel sample rate
    int m_LSM9DS1AccelFsr;                                  // the accel full scale range
    int m_LSM9DS1AccelLpf;                                  // the accel low pass filter

    int m_LSM9DS1CompassSampleRate;                         // the compass sample rate
    int m_LSM9DS1CompassFsr;                                // the compass full scale range
#endif
};

#endif // _RTIMUSETTINGS_H


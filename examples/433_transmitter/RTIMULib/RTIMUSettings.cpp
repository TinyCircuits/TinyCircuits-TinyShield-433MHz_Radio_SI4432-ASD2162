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

#include "RTIMUSettings.h"

#if defined(MPU9150_68) || defined(MPU9150_69)
#include "RTIMUMPU9150.h"
#endif

#if defined(LSM9DS1)
#include "RTIMULSM9DS1.h"
#endif

#define RATE_TIMER_INTERVAL 2

RTIMUSettings::RTIMUSettings()
{
    //  preset general defaults

    m_imuType = -1;
    m_I2CSlaveAddress = 0;

#ifdef MPU9150_68
    //  MPU9150 defaults

    m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;
    m_imuType = RTIMU_TYPE_MPU9150;
    m_I2CSlaveAddress = MPU9150_ADDRESS0;
#endif

#ifdef MPU9150_69
    //  MPU9150 defaults

    m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;
    m_imuType = RTIMU_TYPE_MPU9150;
    m_I2CSlaveAddress = MPU9150_ADDRESS1;
#endif

#ifdef LSM9DS1
    //  LSM9DS1 defaults

    m_LSM9DS1GyroSampleRate = LSM9DS1_GYRO_SAMPLERATE_119;
    m_LSM9DS1GyroBW = LSM9DS1_GYRO_BANDWIDTH_1;
    m_LSM9DS1GyroHpf = LSM9DS1_GYRO_HPF_4;
    m_LSM9DS1GyroFsr = LSM9DS1_GYRO_FSR_500;

    m_LSM9DS1AccelSampleRate = LSM9DS1_ACCEL_SAMPLERATE_119;
    m_LSM9DS1AccelFsr = LSM9DS1_ACCEL_FSR_8;
    m_LSM9DS1AccelLpf = LSM9DS1_ACCEL_LPF_50;

    m_LSM9DS1CompassSampleRate = LSM9DS1_COMPASS_SAMPLERATE_20;
    m_LSM9DS1CompassFsr = LSM9DS1_COMPASS_FSR_4;
    
    m_imuType = RTIMU_TYPE_LSM9DS1;

#endif

}

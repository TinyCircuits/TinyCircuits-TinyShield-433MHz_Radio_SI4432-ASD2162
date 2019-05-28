////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Edited 22 February 2016 by Ben Rose for TinyCircuits, www.tinycircuits.com
//
//  Copyright (c) 2014-2015, richards-tech, LLC
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


#include "RTIMULSM9DS1.h"
#include "RTIMUSettings.h"

#if defined(LSM9DS1)

RTIMULSM9DS1::RTIMULSM9DS1(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMULSM9DS1::~RTIMULSM9DS1()
{
}

bool RTIMULSM9DS1::I2CWriteByte(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  if(Wire.endTransmission())
    return false;
  else
    return true;
}

bool RTIMULSM9DS1::I2CReadByte(uint8_t addr, uint8_t reg, uint8_t *val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission())
    return false;
    
  while(Wire.available())
    Wire.read();
  
  Wire.requestFrom((int)addr, (int)1);

  if (Wire.available()!=1)
    return false;

  *val = Wire.read();
  return true;
}

bool RTIMULSM9DS1::I2CReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *vals){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission())
    return false;
    
  while(Wire.available())
    Wire.read();
  
  Wire.requestFrom((int)addr, (int)count);

  if (Wire.available()!=count)
    return false;

  for (uint8_t i = 0; i < count; i++) {
    *vals = Wire.read();
    vals++;
  }
  return true;
}

int RTIMULSM9DS1::IMUInit()
{
    unsigned char result;

    //  configure IMU

    m_accelGyroSlaveAddr=0;
    
    // work out accel/gyro address
    
    I2CReadByte(LSM9DS1_ADDRESS0, LSM9DS1_WHO_AM_I, &result);
    if (result == LSM9DS1_ID)
        m_accelGyroSlaveAddr = LSM9DS1_ADDRESS0;
    result=0;
    I2CReadByte(LSM9DS1_ADDRESS1, LSM9DS1_WHO_AM_I, &result);
    if (result == LSM9DS1_ID)
        m_accelGyroSlaveAddr = LSM9DS1_ADDRESS1;
    
    if(!m_accelGyroSlaveAddr)
      return -1;

    // work out mag address
    
    m_magSlaveAddr=0;
    
    I2CReadByte(LSM9DS1_MAG_ADDRESS0, LSM9DS1_MAG_WHO_AM_I, &result);
    if (result == LSM9DS1_MAG_ID)
        m_magSlaveAddr = LSM9DS1_MAG_ADDRESS0;
    result=0;
    I2CReadByte(LSM9DS1_MAG_ADDRESS1, LSM9DS1_MAG_WHO_AM_I, &result);
    if (result == LSM9DS1_MAG_ID)
        m_magSlaveAddr = LSM9DS1_MAG_ADDRESS1;
        
    if(!m_magSlaveAddr)
      return -2;

    setCalibrationData();

    //  Set up the gyro/accel

    if (!I2CWriteByte(m_accelGyroSlaveAddr, LSM9DS1_CTRL8, 0x80|0x04))
        return -3;

    delay(100);

    if (!setGyroSampleRate())
        return -4;

    if (!setGyroCTRL3())
        return -5;

    //  Set up the mag

    if (!setAccelCTRL6())
        return -6;

    if (!setAccelCTRL7())
        return -7;

    if (!setCompassCTRL1())
        return -8;

    if (!setCompassCTRL2())
        return -9;

    if (!setCompassCTRL3())
        return -10;

    gyroBiasInit();

    return 0;
}

bool RTIMULSM9DS1::setGyroSampleRate()
{
    unsigned char ctrl1;

    switch (m_settings->m_LSM9DS1GyroSampleRate) {
    case LSM9DS1_GYRO_SAMPLERATE_14_9:
        ctrl1 = 0x20;
        m_sampleRate = 15;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_59_5:
        ctrl1 = 0x40;
        m_sampleRate = 60;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_119:
        ctrl1 = 0x60;
        m_sampleRate = 119;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_238:
        ctrl1 = 0x80;
        m_sampleRate = 238;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_476:
        ctrl1 = 0xa0;
        m_sampleRate = 476;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_952:
        ctrl1 = 0xc0;
        m_sampleRate = 952;
        break;

    default:
        //HAL_ERROR1("Illegal LSM9DS1 gyro sample rate code %d\n", m_settings->m_LSM9DS1GyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_LSM9DS1GyroBW) {
    case LSM9DS1_GYRO_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case LSM9DS1_GYRO_BANDWIDTH_1:
        ctrl1 |= 0x01;
        break;

    case LSM9DS1_GYRO_BANDWIDTH_2:
        ctrl1 |= 0x02;
        break;

    case LSM9DS1_GYRO_BANDWIDTH_3:
        ctrl1 |= 0x03;
        break;
    }

    switch (m_settings->m_LSM9DS1GyroFsr) {
    case LSM9DS1_GYRO_FSR_250:
        ctrl1 |= 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM9DS1_GYRO_FSR_500:
        ctrl1 |= 0x08;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM9DS1_GYRO_FSR_2000:
        ctrl1 |= 0x18;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        //HAL_ERROR1("Illegal LSM9DS1 gyro FSR code %d\n", m_settings->m_LSM9DS1GyroFsr);
        return false;
    }
    return I2CWriteByte(m_accelGyroSlaveAddr, LSM9DS1_CTRL1, ctrl1);
}

bool RTIMULSM9DS1::setGyroCTRL3()
{
    unsigned char ctrl3;

    if ((m_settings->m_LSM9DS1GyroHpf < LSM9DS1_GYRO_HPF_0) || (m_settings->m_LSM9DS1GyroHpf > LSM9DS1_GYRO_HPF_9)) {
        //HAL_ERROR1("Illegal LSM9DS1 gyro high pass filter code %d\n", m_settings->m_LSM9DS1GyroHpf);
        return false;
    }
    ctrl3 = m_settings->m_LSM9DS1GyroHpf;

    //  Turn on hpf
    ctrl3 |= 0x40;

    return I2CWriteByte(m_accelGyroSlaveAddr,  LSM9DS1_CTRL3, ctrl3);
}

bool RTIMULSM9DS1::setAccelCTRL6()
{
    unsigned char ctrl6;

    if ((m_settings->m_LSM9DS1AccelSampleRate < 0) || (m_settings->m_LSM9DS1AccelSampleRate > 6)) {
        //HAL_ERROR1("Illegal LSM9DS1 accel sample rate code %d\n", m_settings->m_LSM9DS1AccelSampleRate);
        return false;
    }

    ctrl6 = (m_settings->m_LSM9DS1AccelSampleRate << 5);

    if ((m_settings->m_LSM9DS1AccelLpf < 0) || (m_settings->m_LSM9DS1AccelLpf > 3)) {
        //HAL_ERROR1("Illegal LSM9DS1 accel low pass fiter code %d\n", m_settings->m_LSM9DS1AccelLpf);
        return false;
    }

    switch (m_settings->m_LSM9DS1AccelFsr) {
    case LSM9DS1_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM9DS1_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM9DS1_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.000244;
        break;

    case LSM9DS1_ACCEL_FSR_16:
        m_accelScale = (RTFLOAT)0.000732;
        break;

    default:
        //HAL_ERROR1("Illegal LSM9DS1 accel FSR code %d\n", m_settings->m_LSM9DS1AccelFsr);
        return false;
    }

    ctrl6 |= (m_settings->m_LSM9DS1AccelLpf) | (m_settings->m_LSM9DS1AccelFsr << 3);

    return I2CWriteByte(m_accelGyroSlaveAddr,  LSM9DS1_CTRL6, ctrl6);
}

bool RTIMULSM9DS1::setAccelCTRL7()
{
    unsigned char ctrl7;

    ctrl7 = 0x00;
    //Bug: Bad things happen.
    //ctrl7 = 0x05;

    return I2CWriteByte(m_accelGyroSlaveAddr,  LSM9DS1_CTRL7, ctrl7);
}


bool RTIMULSM9DS1::setCompassCTRL1()
{
    unsigned char ctrl1;

    if ((m_settings->m_LSM9DS1CompassSampleRate < 0) || (m_settings->m_LSM9DS1CompassSampleRate > 5)) {
        //HAL_ERROR1("Illegal LSM9DS1 compass sample rate code %d\n", m_settings->m_LSM9DS1CompassSampleRate);
        return false;
    }

    ctrl1 = (m_settings->m_LSM9DS1CompassSampleRate << 2);
    
    return I2CWriteByte(m_magSlaveAddr, LSM9DS1_MAG_CTRL1, ctrl1);
}

bool RTIMULSM9DS1::setCompassCTRL2()
{
    unsigned char ctrl2;

    //  convert FSR to uT

    switch (m_settings->m_LSM9DS1CompassFsr) {
    case LSM9DS1_COMPASS_FSR_4:
        ctrl2 = 0;
        m_compassScale = (RTFLOAT)0.014;
        break;

    case LSM9DS1_COMPASS_FSR_8:
        ctrl2 = 0x20;
        m_compassScale = (RTFLOAT)0.029;
        break;

    case LSM9DS1_COMPASS_FSR_12:
        ctrl2 = 0x40;
        m_compassScale = (RTFLOAT)0.043;
        break;

    case LSM9DS1_COMPASS_FSR_16:
        ctrl2 = 0x60;
        m_compassScale = (RTFLOAT)0.058;
        break;

    default:
        //HAL_ERROR1("Illegal LSM9DS1 compass FSR code %d\n", m_settings->m_LSM9DS1CompassFsr);
        return false;
    }

    return I2CWriteByte(m_magSlaveAddr, LSM9DS1_MAG_CTRL2, ctrl2);
}

bool RTIMULSM9DS1::setCompassCTRL3()
{
     return I2CWriteByte(m_magSlaveAddr,  LSM9DS1_MAG_CTRL3, 0x00);
}

int RTIMULSM9DS1::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMULSM9DS1::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!I2CReadByte(m_accelGyroSlaveAddr, LSM9DS1_STATUS, &status))
        return false;

    if ((status & 0x3) == 0)
        return false;

  if (!I2CReadBytes(m_accelGyroSlaveAddr, LSM9DS1_OUT_X_L_G, 6, gyroData))
    return false;

  if (!I2CReadBytes(m_accelGyroSlaveAddr, LSM9DS1_OUT_X_L_XL, 6, accelData))
    return false;
  
  if (!I2CReadBytes(m_magSlaveAddr, LSM9DS1_MAG_OUT_X_L, 6, compassData))
    return false;
    
    m_timestamp = millis();
    
    RTMath::convertToVector(gyroData, m_gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_compass, m_compassScale, false);

    //  sort out gyro axes and correct for bias

    m_gyro.setZ(-m_gyro.z());

    //  sort out accel data;

    m_accel.setX(-m_accel.x());
    m_accel.setY(-m_accel.y());

    //  sort out compass axes

    m_compass.setX(-m_compass.x());
    m_compass.setZ(-m_compass.z());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    //calibrateAccel();

    //  now update the filter

    //updateFusion();
    return true;
}

#endif
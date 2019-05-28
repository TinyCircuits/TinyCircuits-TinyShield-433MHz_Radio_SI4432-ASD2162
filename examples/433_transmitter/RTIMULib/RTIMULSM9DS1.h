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


#ifndef _RTIMULSM9DS1_H
#define	_RTIMULSM9DS1_H

#include "RTIMU.h"

//  LSM9DS1

//  I2C Slave Addresses

#define LSM9DS1_ADDRESS0  0x6a
#define LSM9DS1_ADDRESS1  0x6b
#define LSM9DS1_ID        0x68

#define LSM9DS1_MAG_ADDRESS0        0x1c
#define LSM9DS1_MAG_ADDRESS1        0x1e
#define LSM9DS1_MAG_ID              0x3d

//  LSM9DS1 Register map

#define LSM9DS1_ACT_THS             0x04
#define LSM9DS1_ACT_DUR             0x05
#define LSM9DS1_INT_GEN_CFG_XL      0x06
#define LSM9DS1_INT_GEN_THS_X_XL    0x07
#define LSM9DS1_INT_GEN_THS_Y_XL    0x08
#define LSM9DS1_INT_GEN_THS_Z_XL    0x09
#define LSM9DS1_INT_GEN_DUR_XL      0x0A
#define LSM9DS1_REFERENCE_G         0x0B
#define LSM9DS1_INT1_CTRL           0x0C
#define LSM9DS1_INT2_CTRL           0x0D
#define LSM9DS1_WHO_AM_I            0x0F
#define LSM9DS1_CTRL1               0x10
#define LSM9DS1_CTRL2               0x11
#define LSM9DS1_CTRL3               0x12
#define LSM9DS1_ORIENT_CFG_G        0x13
#define LSM9DS1_INT_GEN_SRC_G       0x14
#define LSM9DS1_OUT_TEMP_L          0x15
#define LSM9DS1_OUT_TEMP_H          0x16
#define LSM9DS1_STATUS              0x17
#define LSM9DS1_OUT_X_L_G           0x18
#define LSM9DS1_OUT_X_H_G           0x19
#define LSM9DS1_OUT_Y_L_G           0x1A
#define LSM9DS1_OUT_Y_H_G           0x1B
#define LSM9DS1_OUT_Z_L_G           0x1C
#define LSM9DS1_OUT_Z_H_G           0x1D
#define LSM9DS1_CTRL4               0x1E
#define LSM9DS1_CTRL5               0x1F
#define LSM9DS1_CTRL6               0x20
#define LSM9DS1_CTRL7               0x21
#define LSM9DS1_CTRL8               0x22
#define LSM9DS1_CTRL9               0x23
#define LSM9DS1_CTRL10              0x24
#define LSM9DS1_INT_GEN_SRC_XL      0x26
#define LSM9DS1_STATUS2             0x27
#define LSM9DS1_OUT_X_L_XL          0x28
#define LSM9DS1_OUT_X_H_XL          0x29
#define LSM9DS1_OUT_Y_L_XL          0x2A
#define LSM9DS1_OUT_Y_H_XL          0x2B
#define LSM9DS1_OUT_Z_L_XL          0x2C
#define LSM9DS1_OUT_Z_H_XL          0x2D
#define LSM9DS1_FIFO_CTRL           0x2E
#define LSM9DS1_FIFO_SRC            0x2F
#define LSM9DS1_INT_GEN_CFG_G       0x30
#define LSM9DS1_INT_GEN_THS_XH_G    0x31
#define LSM9DS1_INT_GEN_THS_XL_G    0x32
#define LSM9DS1_INT_GEN_THS_YH_G    0x33
#define LSM9DS1_INT_GEN_THS_YL_G    0x34
#define LSM9DS1_INT_GEN_THS_ZH_G    0x35
#define LSM9DS1_INT_GEN_THS_ZL_G    0x36
#define LSM9DS1_INT_GEN_DUR_G       0x37

//  Gyro sample rate defines

#define LSM9DS1_GYRO_SAMPLERATE_14_9    0
#define LSM9DS1_GYRO_SAMPLERATE_59_5    1
#define LSM9DS1_GYRO_SAMPLERATE_119     2
#define LSM9DS1_GYRO_SAMPLERATE_238     3
#define LSM9DS1_GYRO_SAMPLERATE_476     4
#define LSM9DS1_GYRO_SAMPLERATE_952     5

//  Gyro banwidth defines

#define LSM9DS1_GYRO_BANDWIDTH_0    0
#define LSM9DS1_GYRO_BANDWIDTH_1    1
#define LSM9DS1_GYRO_BANDWIDTH_2    2
#define LSM9DS1_GYRO_BANDWIDTH_3    3

//  Gyro FSR defines

#define LSM9DS1_GYRO_FSR_250        0
#define LSM9DS1_GYRO_FSR_500        1
#define LSM9DS1_GYRO_FSR_2000       3

//  Gyro high pass filter defines

#define LSM9DS1_GYRO_HPF_0          0
#define LSM9DS1_GYRO_HPF_1          1
#define LSM9DS1_GYRO_HPF_2          2
#define LSM9DS1_GYRO_HPF_3          3
#define LSM9DS1_GYRO_HPF_4          4
#define LSM9DS1_GYRO_HPF_5          5
#define LSM9DS1_GYRO_HPF_6          6
#define LSM9DS1_GYRO_HPF_7          7
#define LSM9DS1_GYRO_HPF_8          8
#define LSM9DS1_GYRO_HPF_9          9

//  Mag Register Map

#define LSM9DS1_MAG_OFFSET_X_L      0x05
#define LSM9DS1_MAG_OFFSET_X_H      0x06
#define LSM9DS1_MAG_OFFSET_Y_L      0x07
#define LSM9DS1_MAG_OFFSET_Y_H      0x08
#define LSM9DS1_MAG_OFFSET_Z_L      0x09
#define LSM9DS1_MAG_OFFSET_Z_H      0x0A
#define LSM9DS1_MAG_WHO_AM_I        0x0F
#define LSM9DS1_MAG_CTRL1           0x20
#define LSM9DS1_MAG_CTRL2           0x21
#define LSM9DS1_MAG_CTRL3           0x22
#define LSM9DS1_MAG_CTRL4           0x23
#define LSM9DS1_MAG_CTRL5           0x24
#define LSM9DS1_MAG_STATUS          0x27
#define LSM9DS1_MAG_OUT_X_L         0x28
#define LSM9DS1_MAG_OUT_X_H         0x29
#define LSM9DS1_MAG_OUT_Y_L         0x2A
#define LSM9DS1_MAG_OUT_Y_H         0x2B
#define LSM9DS1_MAG_OUT_Z_L         0x2C
#define LSM9DS1_MAG_OUT_Z_H         0x2D
#define LSM9DS1_MAG_INT_CFG         0x30
#define LSM9DS1_MAG_INT_SRC         0x31
#define LSM9DS1_MAG_INT_THS_L       0x32
#define LSM9DS1_MAG_INT_THS_H       0x33

//  Accel sample rate defines

#define LSM9DS1_ACCEL_SAMPLERATE_14_9    1
#define LSM9DS1_ACCEL_SAMPLERATE_59_5    2
#define LSM9DS1_ACCEL_SAMPLERATE_119     3
#define LSM9DS1_ACCEL_SAMPLERATE_238     4
#define LSM9DS1_ACCEL_SAMPLERATE_476     5
#define LSM9DS1_ACCEL_SAMPLERATE_952     6

//  Accel FSR

#define LSM9DS1_ACCEL_FSR_2     0
#define LSM9DS1_ACCEL_FSR_16    1
#define LSM9DS1_ACCEL_FSR_4     2
#define LSM9DS1_ACCEL_FSR_8     3

//  Accel filter bandwidth

#define LSM9DS1_ACCEL_LPF_408   0
#define LSM9DS1_ACCEL_LPF_211   1
#define LSM9DS1_ACCEL_LPF_105   2
#define LSM9DS1_ACCEL_LPF_50    3

//  Compass sample rate defines

#define LSM9DS1_COMPASS_SAMPLERATE_0_625    0
#define LSM9DS1_COMPASS_SAMPLERATE_1_25     1
#define LSM9DS1_COMPASS_SAMPLERATE_2_5      2
#define LSM9DS1_COMPASS_SAMPLERATE_5        3
#define LSM9DS1_COMPASS_SAMPLERATE_10       4
#define LSM9DS1_COMPASS_SAMPLERATE_20       5
#define LSM9DS1_COMPASS_SAMPLERATE_40       6
#define LSM9DS1_COMPASS_SAMPLERATE_80       7

//  Compass FSR

#define LSM9DS1_COMPASS_FSR_4   0
#define LSM9DS1_COMPASS_FSR_8   1
#define LSM9DS1_COMPASS_FSR_12  2
#define LSM9DS1_COMPASS_FSR_16  3


//  Define this symbol to use cache mode

//#define LSM9DS1_CACHE_MODE   // not reliable at the moment

#ifdef LSM9DS1_CACHE_MODE

//  Cache defs

#define LSM9DS1_FIFO_CHUNK_SIZE    6                       // 6 bytes of gyro data
#define LSM9DS1_FIFO_THRESH        16                      // threshold point in fifo
#define LSM9DS1_CACHE_BLOCK_COUNT  16                      // number of cache blocks

typedef struct
{
    unsigned char data[LSM9DS1_FIFO_THRESH * LSM9DS1_FIFO_CHUNK_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char accel[6];                                 // the raw accel readings for the block
    unsigned char compass[6];                               // the raw compass readings for the block

} LSM9DS1_CACHE_BLOCK;

#endif

class RTIMULSM9DS1 : public RTIMU
{
public:
    RTIMULSM9DS1(RTIMUSettings *settings);
    ~RTIMULSM9DS1();

    virtual const char *IMUName() { return "LSM9DS1"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM9DS1; }
    virtual int IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool I2CWriteByte(uint8_t addr, uint8_t reg, uint8_t val);
    bool I2CReadByte(uint8_t addr, uint8_t reg, uint8_t *val);
    bool I2CReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *vals);
    bool setGyroSampleRate();
    bool setGyroCTRL3();
    bool setAccelCTRL6();
    bool setAccelCTRL7();
    bool setCompassCTRL1();
    bool setCompassCTRL2();
    bool setCompassCTRL3();

    unsigned char m_accelGyroSlaveAddr;                     // I2C address of accel andgyro
    unsigned char m_magSlaveAddr;                           // I2C address of mag

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;

#ifdef LSM9DS1_CACHE_MODE
    bool m_firstTime;                                       // if first sample

    LSM9DS1_CACHE_BLOCK m_cache[LSM9DS1_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif
};

#endif // _RTIMULSM9DS1_H
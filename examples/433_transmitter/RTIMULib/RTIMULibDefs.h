////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
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

#ifndef _RTIMULIBDEFS_H
#define	_RTIMULIBDEFS_H

#include "RTMath.h"

//  IMU enable defs - only one should be enabled, the rest commented out

//#define MPU9150_68                      // MPU9150 at address 0x68
//#define MPU9150_69                      // MPU9150 at address 0x69
#define LSM9DS1                         // LSM9DS1

//  IMU type codes

#define RTIMU_TYPE_MPU9150                  1                   // InvenSense MPU9150
#define RTIMU_TYPE_LSM9DS1                  8                   // STM LSM9DS1

//  IMU Axis rotation defs
//
//  These allow the IMU to be virtually repositioned if it is in a non-standard configuration
//  Standard configuration is X pointing at north, Y pointing east and Z pointing down
//  with the IMU horizontal. There are 24 different possible orientations as defined
//  below. Setting the axis rotation code to non-zero values performs the repositioning.
//
//  Uncomment the one required

#define RTIMU_XNORTH_YEAST              0                   // this is the default identity matrix
//#define RTIMU_XEAST_YSOUTH              1
//#define RTIMU_XSOUTH_YWEST              2
//#define RTIMU_XWEST_YNORTH              3
//#define RTIMU_XNORTH_YWEST              4
//#define RTIMU_XEAST_YNORTH              5
//#define RTIMU_XSOUTH_YEAST              6
//#define RTIMU_XWEST_YSOUTH              7
//#define RTIMU_XUP_YNORTH                8
//#define RTIMU_XUP_YEAST                 9
//#define RTIMU_XUP_YSOUTH                10
//#define RTIMU_XUP_YWEST                 11
//#define RTIMU_XDOWN_YNORTH              12
//#define RTIMU_XDOWN_YEAST               13
//#define RTIMU_XDOWN_YSOUTH              14
//#define RTIMU_XDOWN_YWEST               15
//#define RTIMU_XNORTH_YUP                16
//#define RTIMU_XEAST_YUP                 17
//#define RTIMU_XSOUTH_YUP                18
//#define RTIMU_XWEST_YUP                 19
//#define RTIMU_XNORTH_YDOWN              20
//#define RTIMU_XEAST_YDOWN               21
//#define RTIMU_XSOUTH_YDOWN              22
//#define RTIMU_XWEST_YDOWN               23

#endif // _RTIMULIBDEFS_H

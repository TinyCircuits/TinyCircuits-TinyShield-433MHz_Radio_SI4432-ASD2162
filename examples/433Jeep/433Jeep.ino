#define use433 1


#include <Wire.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RH_RF22.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object


#include <MotorDriver.h>
#include <ServoDriver.h>

MotorDriver motor(NO_R_REMOVED);
ServoDriver servo(R1_REMOVED);

#if(use433)
RH_RF22 nrf24(7, 3);
#else
RH_NRF24 nrf24(9, 7);
#endif

long timeout = 1000;
long lastcommand = 0;

int servoHold = 2000;
int servoRelease = 1000;

int maxPWM = 500;

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif


void setup()
{
  SerialUSB.begin(115200);
  //while (!SerialUSB);
  Wire.begin();

#if(use433)
  nrf24.init();
  nrf24.setTxPower(RH_RF22_TXPOW_20DBM);
  nrf24.setModemConfig(RH_RF22::GFSK_Rb125Fd125);//GFSK_Rb4_8Fd45?
#else
  nrf24.init();
  nrf24.setChannel(1);
  nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm);
#endif


  if ( motor.begin(500)) {
    SerialMonitorInterface.println("Motor driver not detected!");
    //while (1);
  }
  motor.setMotor(1, 0);
  motor.setMotor(2, 0);
  if (servo.begin(20000)) {    //Set the period to 20000us or 20ms, correct for driving most servos
    SerialMonitorInterface.println("Servo driver not detected!");
    //while (1);
  }
  servo.begin(5000);
  servo.setServo(1, 1500);

  /*
    Wire.beginTransmission(0x70);
    Wire.write(0x04 + 0);
    Wire.endTransmission();
  */
  motor.setFailsafe(100);
  imu = RTIMU::createIMU(&settings);
  imu->IMUInit();
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
}

int currentXdata = 0;
int currentYdata = 0;

int setPoint = 512;
int in = 0;
int lastPWM = 0;

void loop()
{
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    currentXdata = fusion.getFusionPose().x() * RTMATH_RAD_TO_DEGREE;
    currentYdata = -fusion.getFusionPose().y() * RTMATH_RAD_TO_DEGREE;
    //SerialUSB.print(currentXdata);
    //SerialUSB.print('\t');
    //SerialUSB.println(currentYdata);
    
  }
  if (nrf24.available())
  {
    // Should be a message for us now
    uint8_t buf[32];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len) && buf[0] == 5)
    {
      lastcommand = millis();
      int16_t LY = buf[1] << 8 | buf[2];
      int16_t RY = buf[3] << 8 | buf[4];
      int16_t LX = buf[5] << 8 | buf[6];
      int16_t RX = buf[7] << 8 | buf[8];

      if (abs(LY) < 50) {
        LY = 0;
      }
      //motor.setMotor(1, constrain(-LY, -500, 500));
      servo.setServo(1, 1500 - 75 + constrain(1.1 * -(float)RX, -600, 500));

      motor.setMotor(1, constrain(-RY, -500, 500));
      motor.setMotor(2, constrain(LY, -500, 500));


      int voltage = getVCClevel();
      uint8_t data[8];
      data[0] = 5;
      data[1] = abs(0);
      data[2] = voltage >> 8;
      data[3] = voltage;
      data[4] = currentXdata >> 8;
      data[5] = currentXdata;
      data[6] = currentYdata >> 8;
      data[7] = currentYdata;
      nrf24.send(data, 8);
      nrf24.waitPacketSent();
      nrf24.sleep();
    }
  }
  if (millis() > lastcommand + timeout) {
    //digitalWrite(13,LOW);
    motor.setMotor(1, 0);
    motor.setMotor(2, 0);
    lastcommand = millis();
  }

}

int getVCClevel() {
  //http://forum.arduino.cc/index.php?topic=133907.0
  const long InternalReferenceVoltage = 1100L;
  /*ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
    delay(1);
    ADCSRA |= _BV( ADSC );
    while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
    int result = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L;*/
  return 1;//this should be in hundredths of a volt
}

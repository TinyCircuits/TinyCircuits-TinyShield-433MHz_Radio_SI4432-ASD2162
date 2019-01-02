//-------------------------------------------------------------------------------
//  TinyCircuits TinyScreen Robot Control Kit Example Sketch
//  Last Updated 29 April 2015
//  
//  This demo uses a processor board, 433 Mhz radio, and motor x4 board to
//  control a small tank steering/differential steering robot based on data
//  received from the Robot Control Kit Transmitter example. It also sends back
//  some basic telemetry data. The motor board's VM should be connected to a
//  battery, and motors should be soldered to Motor2 and Motor4.
//
//  Written by Ben Rose, TinyCircuits http://Tiny-Circuits.com
//
//-------------------------------------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include <TinyScreen.h>
#include "RH_RF22.h"

#include <SoftPWM.h>

RH_RF22 rf22(7,3);

byte motorSleepPin = A3; 
byte motorIn1Pin[4]={3,5,6,9};
byte motorIn2Pin[4]={2,4,7,8};
int motorRamp[4]={100,100,100,100};//0 to 4000 (time in milliseconds) for each motor
//'ramp' value is how long it will take for the output to reach full power from zero

unsigned long timeout=100;
unsigned long lastcommand=0;

void setup() 
{
  Serial.begin(115200);
  rf22.init();
  rf22.setTxPower(RH_RF22_TXPOW_20DBM);
  rf22.setModemConfig(RH_RF22::GFSK_Rb125Fd125);
  SoftPWMBegin();
  pinMode(motorSleepPin , OUTPUT);
  pinMode(13 , OUTPUT);
  digitalWrite(motorSleepPin , HIGH);
  //the PWM library takes care of setting the other pins to output
}

void loop()
{
  if (rf22.available())
  {
    lastcommand=millis();
    uint8_t buf[RH_RF22_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf22.recv(buf, &len))
    {
      int left=buf[0]<<8 | buf[1];//once we have a new packet, unpack the
      int right=buf[2]<<8 | buf[3];//motor control integers
      if(abs(left)<100)left=0;//add a big deadband for the analog control
      if(abs(right)<100)right=0;//so the motors don't buzz when sitting
      left/=(-2);
      right/=2;
      setMotor(2,left);
      setMotor(4,right);
      uint8_t data[3];
      int voltage=getVCClevel();
      data[0]=abs(rf22.lastRssi());//pack the RSSI and voltage
      data[1]=voltage>>8;
      data[2]=voltage;
      rf22.send(data, 3);//send return packet ot transmitter
      rf22.waitPacketSent();
    }
  }
  if(millis()-lastcommand>timeout){
    setMotor(2,0);//if there hasne't been a packet during timeout, stop motors
    setMotor(4,0);
    lastcommand=millis();
  }
  
}
  
void setMotor(byte motor, int val)
{
  if(motor<1 || motor>4) return;
  motor-=1;
  val=constrain(val,-255,255);
  if(val>0){
    SoftPWMSetFadeTime(motorIn1Pin[motor], motorRamp[motor], motorRamp[motor]);
    SoftPWMSetFadeTime(motorIn2Pin[motor], 0, 0);
    SoftPWMSet(motorIn2Pin[motor],0);
    SoftPWMSet(motorIn1Pin[motor],val);
  }else if(val<0){
    SoftPWMSetFadeTime(motorIn1Pin[motor], 0, 0);
    SoftPWMSetFadeTime(motorIn2Pin[motor], motorRamp[motor], motorRamp[motor]);
    SoftPWMSet(motorIn1Pin[motor],0);
    SoftPWMSet(motorIn2Pin[motor],abs(val));
  }else{
    SoftPWMSetFadeTime(motorIn2Pin[motor], motorRamp[motor], motorRamp[motor]);
    SoftPWMSetFadeTime(motorIn1Pin[motor], motorRamp[motor], motorRamp[motor]);
    SoftPWMSet(motorIn1Pin[motor],0);
    SoftPWMSet(motorIn2Pin[motor],0);
  }; 
}

int getVCClevel(){
  //http://forum.arduino.cc/index.php?topic=133907.0
  const long InternalReferenceVoltage = 1100L;
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  delay(1);
  ADCSRA |= _BV( ADSC );
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
  int result = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L;
  return result;//this should be in hundredths of a volt
}
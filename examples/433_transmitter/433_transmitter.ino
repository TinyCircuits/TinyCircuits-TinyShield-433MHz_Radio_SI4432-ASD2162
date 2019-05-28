#define use433 1

#include <Wire.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RH_RF22.h>
#include <TinyScreen.h>

#define	BLACK           0x00
#define	BLUE            0xE0
#define	RED             0x03
#define	GREEN           0x1C
#define	DGREEN           0x0C
#define YELLOW          0x1F
#define WHITE           0xFF
#define ALPHA           0xFE
#define	BROWN           0x32

TinyScreen display = TinyScreen(TinyScreenPlus);


#if(use433==1)
RH_RF22 nrf24(7, 3);
#else
RH_NRF24 nrf24(9, 7);
#endif


int RX = 0;
int RY = 0;
int LX = 0;
int LY = 0;
int RX_center = 0;
int RY_center = 0;
int LX_center = 0;
int LY_center = 0;
byte leftButton = 0;
byte rightButton = 0;
byte firstRun = 1;

const int amtAverage = 30;
int avgIndex = 0;
float average[amtAverage];

void setup()
{
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  Serial.begin(9600);
  Wire.begin();
  display.begin();
  display.setBrightness(5);
  display.setFont(liberationSansNarrow_8ptFontInfo);
  display.setCursor(0, 0);
  display.print("RSSI: ");
  display.setCursor(0, 10);
  display.print("Voltage: ");
  display.setCursor(0, 20);
#if(use433==1)
  nrf24.init();
  nrf24.setTxPower(RH_RF22_TXPOW_20DBM);
  nrf24.setModemConfig(RH_RF22::GFSK_Rb125Fd125);//GFSK_Rb4_8Fd45?
#else
  nrf24.init();
  nrf24.setChannel(1);
  nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm);
#endif
  display.drawLine(28, 47, 67, 47, WHITE);
}

void loop()
{
  uint8_t data[10];//transmit data
  getJoystick();//update joystick position variables
  if (0) {
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    LX = -(analogRead(3) - 512) - 75;
    LY = -(analogRead(2) - 512) - 50;
    RX = (-(analogRead(0) - 512) + 25 - 78);
    RY = -(analogRead(1) - 512) - 50;

  }
  if (0) {
    //LY = map(analogRead(1), 260, 425, -500, 500) + 60 + 20;
    //RX = map(analogRead(0), 322, 608, -500, 500);
    RX = map(analogRead(1), 470, 860, -300, 300) + 35;
    LY = map(analogRead(0), 360, 602, -500, 500);
    // LY = analogRead(0);
    //RX = analogRead(1);

  }
  if (0) {
    //RY=-RY;
    //LX=LY;
    LX=RY;
    RY=-LY;
  }
  if (0) {
    //RY=-RY;
    //LX=LY;
    LY=-LY;
    RX=RX;
  }
  if (1) {
    //RY=-RY;
    //LX=LY;
    //LY=-LY;
    RY=RX;
  }
  /*SerialUSB.print(LX);
    SerialUSB.print('\t');
    SerialUSB.print(LY);
    SerialUSB.print('\t');
    SerialUSB.print(RX);
    SerialUSB.print('\t');
    SerialUSB.print(RY);
    SerialUSB.println();
  */
  SerialUSB.print(LY);
  SerialUSB.print('\t');
  SerialUSB.print(RX);
  SerialUSB.println();
  data[0] = 5;
  data[1] = LY >> 8; //pack integer data into bytes
  data[2] = LY;
  data[3] = RY >> 8;
  data[4] = RY;
  data[5] = LX >> 8; //pack integer data into bytes
  data[6] = LX;
  data[7] = RX >> 8;
  data[8] = RX;
  data[9] = leftButton;
  
  nrf24.send(data, 10);
  nrf24.waitPacketSent();

  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (nrf24.waitAvailableTimeout(50))
  {
    if (nrf24.recv(buf, &len) && buf[0] == 5)
    {
      uint16_t voltage = buf[2] << 8 | buf[3];
      int16_t x = buf[4] << 8 | buf[5];
      int16_t y = buf[6] << 8 | buf[7];
      Serial.println(x);
      display.setCursor(75, 0);
      display.print(0 - buf[1]);
      display.print("  ");
      display.setCursor(75, 10);
      average[avgIndex++] = voltage / 100.0;
      if (avgIndex >= amtAverage)avgIndex = 0;
      float temp = 0.0;
      for (int i = 0; i < amtAverage; i++) {
        temp += average[i];
      }
      temp /= (float)amtAverage;

      display.print(temp);
      display.print("  ");
      x = constrain(x, -45, 45) / 3;
      y = constrain(y, -45, 45) / 3;
      display.clearWindow(32, 32, 32, 32);
      //display.drawLine(0,47+x,31,47-x,WHITE);
      display.drawLine(32, constrain(47 + y + x, 32, 63), 63, constrain(47 - y + x, 32, 63), WHITE);

    }
    nrf24.recv(buf, &len);
  }
  delay(10);
}

void getJoystick() {
  Wire.requestFrom(0x22, 6);
  int data[4];
  for (int i = 0; i < 4; i++) {
    data[i] = Wire.read();
  }
  byte lsb = Wire.read();
  byte buttons = ~Wire.read();
  leftButton = buttons & 4;
  rightButton = buttons & 8;
  for (int i = 0; i < 4; i++) {
    data[i] <<= 2;
    data[i] |= ((lsb >> (i * 2)) & 3);
    data[i] -= 511;
  }
  RY = data[0]; //was RX
  RX = -data[1]; //was RY
  LX = -data[2];
  LY = data[3];
}

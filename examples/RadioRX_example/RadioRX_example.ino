/*
  TinyDuino Radio Receiver TinyShield Example Sketch
  
  This example is to establish communications with another
  radio of the same type. The other processor and radio TinyDuino
  stack should have the RadioTX_example.ino Radio Transmitter
  TinyShield Example Sketch.

  If using the 433MHz radio: set use433 to a 1 in line 24
  If using the NRF24L01 radio: set use433 to a 0 in line 24
  
  Written 04 June 2019
  By Hunter Hykes
  Modified 
  By 
  
  https://TinyCircuits.com
*/

#include <SPI.h>
#include <RH_RF22.h>
#include <RH_NRF24.h>

#define use433 1  //1 if using 433MHz radio, 0 if using NRF24L01

#if(use433)
RH_RF22 nrf24(7, 3);
#else
RH_NRF24 nrf24(9, 7);
#endif

#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

void setup()
{
  SerialMonitorInterface.begin(115200);

  #if(use433)
    if (!nrf24.init()) {
      SerialMonitorInterface.println("init failed");
    }
    
    nrf24.setTxPower(RH_RF22_TXPOW_20DBM);

    if(!nrf24.setModemConfig(RH_RF22::GFSK_Rb125Fd125)) { //GFSK_Rb4_8Fd45?
      SerialMonitorInterface.println("setModemConfig failed");
    }
  #else
    if (!nrf24.init()) {                        // For NRF24L01 radio
      SerialMonitorInterface.println("init failed");
    }
    if(!nrf24.setChannel(1)) {
      SerialMonitorInterface.println("setChannel failed");
    }
    if(!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm)) {
      SerialMonitorInterface.println("setRF failed");
    }
  #endif

  SPI.setClockDivider(4);
}

void loop()
{
  if (nrf24.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (nrf24.recv(buf, &len)) {
      // NRF24::printBuffer("request: ", buf, len);
      SerialMonitorInterface.print("Received: ");
      SerialMonitorInterface.println((char*)buf);
      
      // Send a reply
      uint8_t data[] = "and hello back to you.";
      nrf24.send(data, sizeof(data));
      nrf24.waitPacketSent();
      
      SerialMonitorInterface.println("Sent a reply");
    } else {
      SerialMonitorInterface.println("Receive Failed!");
    }
  }
}

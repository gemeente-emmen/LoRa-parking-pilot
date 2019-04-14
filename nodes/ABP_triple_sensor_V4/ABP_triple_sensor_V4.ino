/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
//#include "./config_sensor_1.h"
//#include "./config_sensor_2.h"
#include "./config_prototype.h"
#include <Wire.h>
//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiWire.h"

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"

#include <TimeLib.h>
//#include <DS1307RTC.h>

#include <Arduino.h>

#include <MD_DS1307.h>
MD_DS1307 coinCell;


int triggerPin[3] = {12, 6, A3};
int echoPin[3] = {11, 5, A2};
int batVoltPin = 9;
int powerPin = 13;
int indexNewData = 0;
int sensorMessageIndex = 0;
byte messagePort = 0;

int newHistValue = 255;
byte lengthHistory = 5;
byte distanceHistory[3][5] = {
                             {255,255,255, 255, 255},
                             {255,255,255, 255, 255},
                             {255,255,255, 255, 255}};
int sleepcycles = 10;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
//112 == 15 min sleep
bool joined = false;
bool sleeping = false;
#define LedPin 22     // pin 13 LED is not used, because it is connected to the SPI port


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = nwksKey;

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = appsKey;

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = devAddr; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
static osjob_t initjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 1, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            joined = true;
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
          sleeping = true;
          joined = true;


          if (LMIC.dataLen) {
                  // data received in rx slot after tx
                  // if any data received, a LED will blink
                  // this number of times, with a maximum of 10
                  Serial.print(F("Data Received: "));
/*                    Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
                  Serial.print("datalen:    ");
                    Serial.println(LMIC.dataLen);
                  Serial.print("dataBegin:    ");
                    Serial.println(LMIC.dataBeg);
                  Serial.print("data length - 1:   "); 
                    Serial.println(LMIC.frame[LMIC.dataLen - 1]);
                  Serial.print("txrxFlags 1:     ");
                     Serial.println(LMIC.txrxFlags);
                  Serial.print("TXRX_PORT:       ");
                     Serial.println(LMIC.frame[TXRX_PORT]);
                  Serial.print("EV_TXCOMPLETE:    ");
                     Serial.println(LMIC.frame[EV_TXCOMPLETE]);
 */
                  
                  int newSleepCycles = LMIC.frame[LMIC.dataBeg] | LMIC.frame[LMIC.dataBeg + 1] << 8;
            Serial.print("Sleep cycles =    ");
            Serial.println(newSleepCycles);
            sleepcycles = newSleepCycles;
            
            if(LMIC.dataLen >= 2){
              Serial.print("New spreading factor wil be: ");
              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
              switch(LMIC.frame[LMIC.dataBeg + 2]){
                case 7:
    //              Serial.print("setting Spreading factor to 7:  ");
    //              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
                  LMIC_setDrTxpow(DR_SF7,14);
                  break;
                case 8:
    //              Serial.print("setting Spreading factor to 8:  ");
    //              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
                  LMIC_setDrTxpow(DR_SF8,14);
                  break;
                case 9:
    //              Serial.print("setting Spreading factor to 9:  ");
    //              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
                  LMIC_setDrTxpow(DR_SF9,14);
                  break;
                case 10:
    //              Serial.print("setting Spreading factor to 10:  ");
    //              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
                  LMIC_setDrTxpow(DR_SF10,14);
                  break;
                case 11:
    //              Serial.print("setting Spreading factor to 11:  ");
    //              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
                  LMIC_setDrTxpow(DR_SF11,14);
                  break;
                case 12:
    //              Serial.print("setting Spreading factor to 12:  ");
    //              Serial.println(LMIC.frame[LMIC.dataBeg + 2]);
                  LMIC_setDrTxpow(DR_SF12,14);
                  break;
              }
            }
          }

          
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            /*if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            */
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}
int getDistance(int triggerPin, int echoPin){
  for(int cycle = 0; cycle < 10; cycle++){
    int distance = 0;
    int diverance = 0;
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    distance = pulseIn(echoPin, HIGH) * 0.034 / 2;    
    for(int i = 0; i < 4; i++){
      delayMicroseconds(10);
      digitalWrite(triggerPin, LOW);
      delayMicroseconds(2);
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPin, LOW);
      int controlMeasurement = pulseIn(echoPin, HIGH) * 0.034 / 2;
      diverance += distance - controlMeasurement;
      diverance /= 5;
      delay(50);
    }
    if(diverance < 10){
    return distance;
    }else{
      Serial.print("Diverance:");
      Serial.println(diverance);
      Serial.println("Diverance to large measuring again");
      
    }
  }
  return 0;
}

byte condenseInt(int intValue){
/* devides a bit value in halve via integer devision and then flipping the first bit is the original value was odd (instead of even)
 *  this allows to shrink the parking sensor values (max 285) to the size to fit in a single byte instead of 2 byte int.
 */
  if(intValue > 255 * 2){ //The value would not fit in a single byte anyway, so return largest number voor error analysis
    return 255;
  }
  
  if(bitRead(intValue, 0)){
    intValue /= 2;
    bitSet(intValue, 0);
  }else{
    intValue /= 2;
  }
  return intValue;
}

void updatePortNumber(bool checkPass){
  if(checkPass and sensorMessageIndex < 7){
    bitSet(messagePort, sensorMessageIndex);
    sensorMessageIndex++;
  }else if(sensorMessageIndex < 7){
    sensorMessageIndex++;
  }
}

void do_send(osjob_t* j) {
  byte buffer[20];
  // It is a bad idea to send ASCII for real, only use this when developing!!
  int duration, distance;
  long timestamp;
  indexNewData = 0;
  messagePort = 0; 
  sensorMessageIndex = 0;
  digitalWrite(powerPin, HIGH);
  delay(20);
        for(int row = 0; row < 3; row++){
          distance = getDistance(triggerPin[row], echoPin[row]);
          if(distance){
            addShiftHistory(newHistValue, distanceHistory[row], lengthHistory);
            buffer [indexNewData] = distance & 0xff;
            buffer [indexNewData + 1] = distance >> 8 & 0xff;
            indexNewData += 2;
            for(int col= 0; col < lengthHistory; col++){
              buffer[indexNewData] = distanceHistory[row][col] & 0xff;
              indexNewData ++;
            }
            newHistValue = condenseInt(distance);
            updatePortNumber(true);
          }else{
            updatePortNumber(false);
          }
        }
    /*    
    timestamp = RTC.get();
    buffer[indexNewData] = timestamp & 0xFF;
    buffer[indexNewData + 1] = (timestamp >> 8) & 0xFF;
    buffer[indexNewData + 2] = (timestamp >> 16) & 0xFF;
    buffer[indexNewData + 3] = (timestamp >> 24) & 0xFF;
    updatePortNumber(true);
    indexNewData += 4;
    Serial.print("time = ");
    Serial.println(timestamp);
     */
  //indexNewData = getTime(buffer, indexNewData);
  updatePortNumber(false);
  
  int batteryVoltage = analogRead(batVoltPin);
  buffer [indexNewData] = batteryVoltage & 0xFF;
  buffer [indexNewData + 1] = (batteryVoltage >> 8) & 0xFF;
  updatePortNumber(true);
  indexNewData += 2;
  
  //int temp = getTemp();
  //buffer [indexNewData] = temp & 0XFF;
  //buffer [indexNewData + 1] = (temp >> 8) & 0xFF;
  //indexNewData += 2;
  updatePortNumber(false);

    Serial.println("new buffer:");
    for(int i = 0; i < indexNewData; i ++){
      Serial.print(buffer[indexNewData]);
      Serial.print(", ");  
    }
      Serial.print("\n");
      
  
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
    }else{
      // Prepare upstream data transmission at the next possible time.
      if(messagePort != 0 && messagePort < 128){
      Serial.println(F("Sending: "));
      LMIC_setTxData2(messagePort, buffer, indexNewData, 0);
        Serial.print("Port:");
        Serial.println(messagePort);
        updateFrameCount();
      }else{
        Serial.println("Port number in use by TTN not sending");
        Serial.print("Port:");
        Serial.print(messagePort);
        Serial.print("\n");
      } 
    }
    digitalWrite(powerPin, LOW);
}

// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}

int getFrameCount(){
  byte frameBuffer[2];
  coinCell.readRAM(8, frameBuffer, 2);
  int frameCount = frameBuffer[0] << 8 | frameBuffer[1];
  Serial.print("frame count is ");
  Serial.println(frameCount);
  return frameCount;
}

void addShiftHistory(byte newValue, byte historyArray[], byte lengthArray){
  
  for(byte i = lengthArray; i > 0; i--){
    historyArray[i] = historyArray[i - 1]; 
  }
  historyArray[0] = newValue;
}

void updateFrameCount(){
    byte readBuffer[2];
    coinCell.readRAM(8, readBuffer, 2);
    int frameNR = readBuffer[0] << 8 | readBuffer[1];
    frameNR++;
    byte writeBuffer[2];
  writeBuffer[0] = highByte(frameNR);
  writeBuffer[1] = lowByte(frameNR);
  
  coinCell.writeRAM(8, writeBuffer, 2);
}

int getTime(byte *payloadBuffer, int indexNewData){
  coinCell.readTime();
  byte tsSeconds = coinCell.s;
  payloadBuffer[indexNewData] = tsSeconds;
  indexNewData++;
  byte tsMinutes = coinCell.m;
  payloadBuffer[indexNewData] = tsMinutes;
  indexNewData++;
  byte tsHours = coinCell.h;
  payloadBuffer[indexNewData] = tsHours;
  indexNewData++;
  byte tsDay = coinCell.dd;
  payloadBuffer[indexNewData] = tsDay;
  indexNewData++;
  byte tsMonth = coinCell.mm;
  payloadBuffer[indexNewData] = tsMonth;
  indexNewData++;
  return indexNewData;
  }

int getTemp(void){
  unsigned int wADC;
  int t;
  ADMUX = (_BV(REFS1)|_BV(REFS0)|_BV(MUX3));
  ADCSRA |= _BV(ADEN);
  delay(20);
  ADCSRA |= _BV(ADSC);
  while(bit_is_set(ADCSRA, ADSC));
  wADC = ADCW;
  t= (wADC - 324.31 / 1.22);
  return t;
}

void setup() {

  for(int i = 0; i < 3; i++){
    pinMode(triggerPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
    pinMode(powerPin, OUTPUT);

    pinMode(batVoltPin, INPUT);
    Serial.begin(9600);
    Serial.println("Serial connection online:");

  pinMode(12, OUTPUT);
  pinMode(10, OUTPUT);
  
  Wire.begin();
  Wire.setClock(400000L);
    Wire.begin();
  Wire.setClock(400000L);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_setCallback(&initjob, initfunc);
  LMIC_reset();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF8,14);

    digitalWrite(powerPin, HIGH);
    int fc = getFrameCount();
    Serial.println(fc);
    LMIC.seqnoUp = fc;

//    digitalWrite(powerPin, LOW);
    // Start job
    do_send(&sendjob);
}


void loop() {
/*  if(joined == false)
  {
    os_runloop_once();
  }
  else
  {
  */
    //do_send(&sendjob);    // Sent sensor values
    digitalWrite(13, HIGH);
    digitalWrite(10, LOW);
    Serial.print("test 1 sleeping = ");
    Serial.println(sleeping);
      
      
      float batteryVoltage = (analogRead(batVoltPin) * 2 * 3.3) / 1024;
      
      if (batteryVoltage < 3.33){
        //do deepsleep stuff
        messagePort = 0;
      //Will sleep for 92 days
        for (int i = 0; i < 1000000 ;i++)
      {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
        
      }
      
      while(sleeping == false)
      {
        os_runloop_once();
        
      }
      Serial.print("test 2 sleeping = ");
      Serial.println(sleeping);
      sleeping = false;
      for (int i = 0; i < sleepcycles ;i++)
      {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
//   }
      digitalWrite(LedPin,((millis()/100) % 2) && (joined==false)); // only blinking when joining and not sleeping

}

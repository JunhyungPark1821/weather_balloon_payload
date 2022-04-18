aaaaaa

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
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
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 12
#define GPSSerial Serial1

// Setup a oneWire instance to communicate with any OneWire devices  
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

char radiodata[1000] = {};


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x0A, 0xCE, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xCA, 0xE0, 0x9D, 0x08, 0x87, 0xCA, 0x8A, 0xCD, 0x21, 0xA2, 0xF9, 0x8E, 0x14, 0x88, 0x46, 0x7A };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[19];
uint32_t LatitudeBinary, LongitudeBinary;

static osjob_t sendjob;

String completeLat = "";
String longWhole = "";
String longWholeFifth = "";
String longRem = "";
String completeAlt = "";
String fifthDigit = "";
String completeTemperature = "";

double totTime;
double startTime;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, 11},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

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
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println("Time for a transmission: " + String(millis() - startTime));
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            startTime = millis();
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      float gpsLat = completeLat.toFloat();
      float gpsLongWhole = longWhole.toFloat();
      float gpsLongWholeFifth = longWholeFifth.toFloat();
      float gpsLongRem = longRem.toFloat();
      float gpsAlt = completeAlt.toFloat();
      float temperature = completeTemperature.toFloat();
      
//      uint16_t payloadGPSLat = LMIC_f2sflt16(0.12345678);
//      Serial.println (LMIC_f2sflt16(0.1234));
//      Serial.println (LMIC_f2sflt16(0.5678));
//      byte latLow = lowByte(payloadGPSLat);
//      Serial.println (latLow);
//      byte latHigh = highByte(payloadGPSLat);
//      Serial.println (latHigh);
//      mydata[0] = latLow;
//      mydata[1] = latHigh;

//      uint16_t payloadGPSLong = LMIC_f2sflt16(0.5678);
//      byte longLow = lowByte(payloadGPSLong);
//      byte longHigh = highByte(payloadGPSLong);
//      mydata[2] = longLow;
//      mydata[3] = longHigh;
//
//      int exponent = completeAlt.substring(0,completeAlt.lastIndexOf('.')).length();
//      uint16_t payloadGPSAlt = LMIC_f2sflt16(0.69);
//      byte altLow = lowByte(payloadGPSAlt);
//      byte altHigh = highByte(payloadGPSAlt);
//      mydata[4] = altLow;
//      mydata[5] = altHigh;
//
//      uint16_t payloadExp = LMIC_f2sflt16(exponent/10);
//      byte expLow = lowByte(payloadExp);
//      byte expHigh = highByte(payloadExp);
//      mydata[6] = expLow;
//      mydata[7] = expHigh;

      int latWhole = gpsLat;
      uint16_t payloadGPSLatWhole = LMIC_f2sflt16(latWhole/10000.0);
      byte latWholeLow = lowByte(payloadGPSLatWhole);
      byte latWholeHigh = highByte(payloadGPSLatWhole);
      mydata[0] = latWholeLow;
      mydata[1] = latWholeHigh;

      int latRem = (gpsLat - double(latWhole)) * 100000.0;
      uint16_t payloadGPSLatRem = LMIC_f2sflt16(latRem/100000.0);
      byte latRemLow = lowByte(payloadGPSLatRem);
      byte latRemHigh = highByte(payloadGPSLatRem);
      mydata[2] = latRemLow;
      mydata[3] = latRemHigh;
      
      uint16_t payloadGPSLongWhole = LMIC_f2sflt16(gpsLongWhole/10000.0);
      byte longWholeLow = lowByte(payloadGPSLongWhole);
      byte longWholeHigh = highByte(payloadGPSLongWhole);
      mydata[4] = longWholeLow;
      mydata[5] = longWholeHigh;

      uint16_t payloadGPSLongRem = LMIC_f2sflt16(gpsLongRem/10000.0);
      byte longRemLow = lowByte(payloadGPSLongRem);
      byte longRemHigh = highByte(payloadGPSLongRem);
      mydata[6] = longRemLow;
      mydata[7] = longRemHigh;

      int exponent = completeAlt.substring(0,completeAlt.lastIndexOf('.')).length();
      int altWhole = gpsAlt;
      double digit = pow(10,exponent);
      uint16_t payloadGPSAlt = LMIC_f2sflt16(double(altWhole)/digit);
      byte altWholeLow = lowByte(payloadGPSAlt);
      byte altWholeHigh = highByte(payloadGPSAlt);
      mydata[8] = altWholeLow;
      mydata[9] = altWholeHigh;

      uint16_t altExp = LMIC_f2sflt16(exponent/10.0);
      byte expLow = lowByte(altExp);
      byte expHigh = highByte(altExp);
      mydata[10] = expLow;
      mydata[11] = expHigh;

      uint16_t longInd = LMIC_f2sflt16(gpsLongWholeFifth/10.0);
      byte longIndLow = lowByte(longInd);
      byte longIndHigh = highByte(longInd);
      mydata[12] = longIndLow;
      mydata[13] = longIndHigh;

      int tempWhole = temperature;
      Serial.println (tempWhole);
      uint16_t temperatureWhole = LMIC_f2sflt16(tempWhole/1000.0);
      byte temperatureWholeLow = lowByte(temperatureWhole);
      byte temperatureWholeHigh = highByte(temperatureWhole);
      mydata[14] = temperatureWholeLow;
      mydata[15] = temperatureWholeHigh;

      int tempRem = (temperature - double(tempWhole)) * 1000.0;
      Serial.println (tempRem);
      uint16_t temperatureRem = LMIC_f2sflt16(tempRem/1000.0);
      byte temperatureRemLow = lowByte(temperatureRem);
      byte temperatureRemHigh = highByte(temperatureRem);
      mydata[16] = temperatureRemLow;
      mydata[17] = temperatureRemHigh;
      
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
      Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000);
    GPSSerial.begin(9600);
    Serial.begin(9600);
    sensors.begin(); 
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setDrTxpow(DR_SF7,14);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
  char c=0;
  String gpsdata,year;
    
  while(c !=10) {
    os_runloop_once();
     if(Serial1.available()) {
      c=Serial1.read();
      gpsdata+=c;
    }
  }
  
  if(gpsdata.startsWith("$GNGGA,")) {
    gpsdata.replace("$GNGGA,","");
    gpsdata=gpsdata.substring(0,gpsdata.lastIndexOf(','));
    gpsdata=gpsdata.substring(0,gpsdata.lastIndexOf(','));
    gpsdata=gpsdata.substring(0,gpsdata.lastIndexOf(','));
    gpsdata=gpsdata.substring(0,gpsdata.lastIndexOf(','));
    gpsdata=gpsdata.substring(0,gpsdata.lastIndexOf(','));
    
    gpsdata.toCharArray(radiodata, gpsdata.length()+1);

    Serial.println (gpsdata);

    completeLat = "";
    completeAlt = "";
    longWhole = "";
    longWholeFifth = "";
    longRem = "";
    fifthDigit = "";
    completeTemperature = "";

    sensors.requestTemperatures(); // Send the command to get temperature readings 

    completeTemperature = String(sensors.getTempCByIndex(0));

    Serial.println(completeTemperature);

    if (radiodata[15] == ',') {
    } else {
      for (int i = 10; i < 19; i++) {
        completeLat += String(radiodata[i]);        
      }
      for (int i = 23; i < 27; i++) {
        longWhole += String(radiodata[i]);
      }
      longWholeFifth = String(radiodata[27]);
      for (int i = 29; i < 33; i++) {
        longRem += String(radiodata[i]);
      }
      for (int i = 47; i < sizeof(radiodata); i++) {
        completeAlt += String(radiodata[i]);
      }
    }
  }
}

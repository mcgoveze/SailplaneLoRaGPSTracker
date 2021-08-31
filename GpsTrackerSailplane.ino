// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// Modified by github FGCDAM user https://github.com/fcgdam
#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "soc/efuse_reg.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <CayenneLPP.h>
#include <Adafruit_BMP280.h>

#define LEDPIN 2

#define I2C_ADDRESS 0x76

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

const uint8_t vbatPin = 34;
float VBAT;  // battery voltage from ESP32 ADC read

unsigned int counter = 0;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);


/*************************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 * 01 29 60 01 c2 b0 06 f6 00 05 5f d0 08 5a b0 0a
 *************************************/
static u1_t NWKSKEY[16] = { 0x01,0x29 ,0x60 ,0x01 ,0xc2 ,0xb0 ,0x06 ,0xf6 ,0x00 ,0x05,0x5f ,0xd0 ,0x08 ,0x5a ,0xb0 ,0x0a};

//{ 0x0a,0xb0,0x5a ,0x08 ,0xd0 ,0x5f ,0x05 ,0x00 ,0xf6 ,0x06 ,0xb0 ,0xc2 ,0x01 ,0x60 ,0x29 ,0x01 };
//{ 0x01,0x29 ,0x60 ,0x01 ,0xc2 ,0xb0 ,0x06 ,0xf6 ,0x00 ,0x05,0x5f ,0xd0 ,0x08 ,0x5a ,0xb0 ,0x0a};
//{ 0x1d ,0xd1 ,0x6b ,0xa2 ,0x5f ,0x04 ,0x59 ,0xa4 ,0x33 ,0x40 ,0x22 ,0xff ,0xd8 ,0xcc ,0xd1 ,0xb3 };

static u1_t APPSKEY[16] = {0x0a,0xb0,0x5a ,0x08 ,0xd0 ,0x5f ,0x05 ,0x00 ,0xf6 ,0x06 ,0xb0 ,0xc2 ,0x01 ,0x60 ,0x29 ,0x01};

//{ 0x96,0xa8,0x9e,0x91,0xff,0xfe,0x0d,0x7a,0x93,0xeb,0x42,0xe1,0xf1,0x74,0x4c,0xfa };

// { 0x08,0x9d,0x30,0x01,0xa0,0x60,0x03,0xf3,0xe0,0x0a,0xcf,0xc0,0x05,0x9f,0x60,0x09};
//{ 0x09 ,0x60 ,0x9f ,0x05 ,0xc0 ,0xcf ,0x0a ,0xe0 ,0xf3 ,0x03 ,0x60 ,0xa0 ,0x01 ,0x30 ,0x9d ,0x08 };

//{ 0x08,0x9d,0x30,0x01,0xa0,0x60,0x03,0xf3,0xe0,0x0a,0xcf,0xc0,0x05,0x9f,0x60,0x09};
//{ 0x0b ,0xf2 ,0xb6 ,0xe6 ,0x88 ,0xdc ,0x7b ,0x03 ,0x5c ,0x66 ,0x3d ,0x36 ,0x49 ,0xe8 ,0x2c ,0x8d };

static u4_t DEVADDR = 0x006b01b8;   

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 15;
char TTN_response[30];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};


CayenneLPP lpp(51); 

static const uint32_t GPSBaud = 9600;

#define RX2 13
#define TX2 12
static const int RXPin = RX2, TXPin = TX2;
SoftwareSerial gps_serial(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;


//Fly Ranch , Club de Planeadores La Plata
//https://goo.gl/maps/evChWYqz4Mq4Qemm6

static const double GW_LAT = -35.09556, GW_LON = -58.07974;


typedef struct sensorData {
  double temperature;
  double pressure;
};

#define PACKET_SIZE sizeof(sensorData)
typedef union LoRa_Packet {
  sensorData sensor;
  byte LoRaPacketBytes[PACKET_SIZE];
};

LoRa_Packet sensorinfo;

void do_send(osjob_t* j){
    int snd_ack;
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        smartDelay(500);
        lpp.reset();                           // clear the buffer
        lpp.addTemperature(1, sensorinfo.sensor.temperature);
        lpp.addBarometricPressure(2, sensorinfo.sensor.pressure);
    
        smartDelay(0);

        lpp.addGPS(3, gps.location.lat(), gps.location.lng(), gps.altitude.meters());   // channel 3, coordinates
        smartDelay(0);
        //lpp.addDirection(4,gps.course.deg());
        //smartDelay(0);
       // lpp.addAltitude(4,gps.altitude.meters());
        
        smartDelay(0);
        lpp.addAnalogOutput(4,VBAT);
        
        //Distancia al Club de Planeadores
         unsigned long distanceMetersToGW =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      GW_LAT, 
      GW_LON) ;
      
      lpp.addDistance(4,distanceMetersToGW);
      
      printDateTime(gps.date, gps.time);
      
      lpp.addAnalogOutput(5,gps.course.deg());
      lpp.addAnalogOutput(6,gps.altitude.meters());
      lpp.addDigitalOutput(7,distanceMetersToGW);

      LMIC_setTxData2(2, lpp.getBuffer(), lpp.getSize(),snd_ack);

         Serial.print("Satellites: ");
         Serial.println(gps.satellites.value());
         Serial.print("Latitude= "); 
         Serial.print(gps.location.lat(), 6);
         Serial.print(" Longitude= "); 
         Serial.println(gps.location.lng(), 6);
        
        Serial.println(F("Sending uplink packet..."));
        digitalWrite(LEDPIN, HIGH);
        display.clear();
        display.drawString (0, 0, "Sending uplink packet...");
        
        DisplayVoltage(VBAT, true, 24);
        DisplaySatellites( gps.satellites.value(), gps.location.isValid(), 22);
        DisplayDistanceGW(distanceMetersToGW, gps.location.isValid(), 12);
       
        display.drawString (0, 50, String (++counter));
        printDateTime(gps.date, gps.time);
        Serial.print(F("Packet queued for freq: "));
        Serial.println(LMIC.freq);
        display.display ();
        
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        display.clear();
        display.drawString (0, 0, "EV_TXCOMPLETE event!");

        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
          Serial.println(F("Received ack"));
          display.drawString (0, 20, "Received ACK.");
        }

        if (LMIC.dataLen) {
          int i = 0;
          // data received in rx slot after tx
          Serial.print(F("Data Received: "));
          Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          Serial.println();

          display.drawString (0, 20, "Received DATA.");
          for ( i = 0 ; i < LMIC.dataLen ; i++ )
            TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];

          TTN_response[i] = 0;
          display.drawString (0, 32, String(TTN_response));
        }

        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        digitalWrite(LEDPIN, LOW);
        display.drawString (0, 50, String (counter));
        display.display ();
    }
}

int getChipRevision()
{
  return (REG_READ(EFUSE_BLK0_RDATA3_REG) >> (EFUSE_RD_CHIP_VER_REV1_S)&&EFUSE_RD_CHIP_VER_REV1_V) ;
}

void printESPRevision() {
  Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG): ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

  Serial.print("EFUSE_RD_CHIP_VER_REV1_S: ");
  Serial.println(EFUSE_RD_CHIP_VER_REV1_S, BIN);

  Serial.print("EFUSE_RD_CHIP_VER_REV1_V: ");
  Serial.println(EFUSE_RD_CHIP_VER_REV1_V, BIN);

  Serial.println();

  Serial.print("Chip Revision (official version): ");
  Serial.println(getChipRevision());

  Serial.print("Chip Revision from shift Operation ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15, BIN);

}

void setup() {
    Serial.begin(115200);
    delay(1500);   // Give time for the seral monitor to start up
    Serial.println(F("Starting..."));
      pinMode(vbatPin, INPUT);
    printESPRevision();

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

   // reset the OLED
    pinMode(OLED_RESET,OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    display.init ();
    display.flipScreenVertically ();
    display.setFont (ArialMT_Plain_10);

    display.setTextAlignment (TEXT_ALIGN_LEFT);

    display.drawString (0, 0, "Init!");
    display.display ();
    
    if ( DEVADDR == 0x00 )   {  // Device is misconfigured with an invalid or not defined device id.
        int i = 0;

        while ( true ) {
            display.clear();
            display.drawString (0, 0, "INIT FAILED! ");
            display.drawString (0, 9, "Device ID is not SET!");
          
            display.drawString (0, 18, "Invalid TTN settings!");
            display.drawString (0, 27, "Set NWSKEY and APPSKEY!");
        
            display.drawString( 0, 36, "Correct and reset.");

            display.drawString( 0, 45, "Waiting for user action...");
            if ( i == 0 )
                display.drawString( 0, 54, "." );
            else 
                display.drawString( 0, 54, "*" );
            i++; 
            i = i % 2; 
            display.display ();
            delay( 1000 );
        }

    }

     //init GPS
     //gps_serial.begin(9600, SERIAL_8N1, RX2, TX2);
     gps_serial.begin(GPSBaud);

    initSensor();

    // LMIC init
    os_init();

    display.drawString (0, 8, "LMIC was initiated!");
    display.display ();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    display.drawString (0, 16, "LMIC reset -> OK!");
    display.display ();

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    display.drawString (0, 24, "LMIC setChannels -> OK!");
    display.display ();


    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // First, disable channels 0-7
    for (int channel=0; channel<8; ++channel) {
      LMIC_disableChannel(channel);
    }
    /*// Now, disable channels 16-72 (is there 72 ??)
    for (int channel=16; channel<72; ++channel) {
       LMIC_disableChannel(channel);
    } */ 
    LMIC_selectSubBand(1);


    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF10,14);
    
    
    display.drawString (0, 32, "LMIC setup -> OK!");
    display.display ();

    for ( int i = 0 ; i < 5 ; i++ ) {
        delay ( 1000 );
        
        display.drawString (0+i*8, 50, ".");
        display.display ();
    }

    Serial.println("LMIC setup done.");
    Serial.println("Starting transmit job.");
       makeMeassurement();

    // Start job
    do_send(&sendjob);
}

void loop() {
    makeMeassurement();

    os_runloop_once();
    smartDelay(1000);
   
   //Serial.println(VBAT);
    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));

}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gps_serial.available())
      gps.encode(gps_serial.read());
  } while (millis() - start < ms);
}


static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    display.drawString (0,40, "******** ");
  }
  else
  {
    char sz[32];
    sprintf(sz, "Hora GPS: %02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    display.drawString (0,40,sz);
  }
}


static void DisplayDistanceGW(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "Dist: %ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  display.drawString(0,32,sz);
  smartDelay(0);
}


static void DisplaySatellites(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "Sats: %ld    T1:%.2f", val, sensorinfo.sensor.temperature);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  display.drawString(0,21,sz);
  smartDelay(0);
}


static void DisplayVoltage(float val, bool valid, int len)
{
  char sz[32] = "*****************";

  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    sprintf(sz, "Bat: %.2f P1:%.2f hPa", val, sensorinfo.sensor.pressure);

    display.drawString(0,9, sz );
    
  }
  smartDelay(0);
}



void initSensor()
{
 
  if (!bmp.begin(0x76)) {
    Serial.println(F("error!"));
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
}


void makeMeassurement()
{
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  //Serial.print("Temperature: "); 
  sensorinfo.sensor.temperature = temp_event.temperature;
  sensorinfo.sensor.pressure = pressure_event.pressure;

/* Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.println();
  */
 // Battery Voltage
     VBAT = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;

}


/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BluefruitConfig.h"
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
    /* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
float maxAccel=0;
float maxTilt=0;
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

//Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
  //                    BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
//instance of main sending pin #s
/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*//*modified for bluetooth*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  ble.println("------------------------------------");
  ble.print  ("Sensor:       "); ble.println(sensor.name);
  ble.print  ("Driver Ver:   "); ble.println(sensor.version);
  ble.print  ("Unique ID:    "); ble.println(sensor.sensor_id);
  ble.print  ("Max Value:    "); ble.print(sensor.max_value); ble.println(" xxx");
  ble.print  ("Min Value:    "); ble.print(sensor.min_value); ble.println(" xxx");
  ble.print  ("Resolution:   "); ble.print(sensor.resolution); ble.println(" xxx");
  ble.println("------------------------------------");
  ble.println("");
  delay(1000);
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

void setup(void)
{
//  while (!Serial);  // required for Flora & Micro
 float maxAccel=0;
float maxTilt=0;
 // Serial1.begin(115200);
 // Serial.begin(9600);
  Serial.begin(115200);
  //ble.begin(9600);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false); /*//????????????*/

//  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
 // Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /*new connection*/
  /* Initialise the sensor */
  if(!bno.begin())
  {/*if not hooked up correctly no information will be passed*/
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(500);
    
  bno.setExtCrystalUse(true);
  //Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
 // Serial.println(F("Then Enter characters to send to Bluefruit"));
 // Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
 // while (! ble.isConnected()) {
    //  delay(500); 
      
  //}/*kind of usueful*/

 
  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    ble.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);/*must add constant MODE_LED*?*/
  }

  // Set module to DATA mode
  ble.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  ble.println(F("******************************"));
    displaySensorDetails();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
     Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)

{ /* Get a new sensor event */
  sensors_event_t event; 
  bno.getEvent(&event);
  //115200)
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /* Also send calibration data for each sensor. */
 uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 imu::Vector<3> acelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER );
 imu::Vector<3> gyro1 = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE  );
 //Serial.print(euler.x());
 /*   Serial.print("X accel  "); Serial.print(acelerometer.x(),4);Serial.print('\t');
    Serial.print("Y accel  "); Serial.print(acelerometer.y(),4);Serial.print('\t');
    Serial.print("Z accel  ");Serial.print(acelerometer.z(),4);Serial.print('\t');
    Serial.println();

    */
/*   The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientations: "));//F
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));//F
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));//F
  Serial.print((float)event.orientation.z);
  Serial.println((""));//F
   /* Also send calibration data for each sensor. */
 // uint8_t sys, gyro, accel, mag = 0;
 // bno.getCalibration(&sys, &gyro, &accel, &mag);
  /* Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(F("Calibrationssssss: "));:::
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
  Serial.print('\n');*/
ble.print("Accel ");
   ble.print("X: "); ble.print(acelerometer.x(),2); ble.print('\t');
   ble.print("Y: "); ble.print(acelerometer.y(),2); ble.print('\t');
    ble.print("Z: ");ble.print(acelerometer.z(),2); ble.println();
     ble.print("Degrees/Tilt \t");
    ble.print("X: ");
  ble.print(euler.x());ble.print('\t');
  ble.print(" Y: ");
  ble.print(euler.y());ble.print('\t');
  ble.print(" Z: ");
  ble.print(euler.z());ble.print('\t');
  ble.print("\t\t");
  ble.println();
  delay(200);
   //Serial1.print("Hey Im tryiing to work");
  /* Display the floating point data */
  ble.print("Gyroscope");ble.print('\t');
  ble.print("X: ");
  ble.print(gyro1.x(),2);
  ble.print('\t');
   ble.print("Y: ");
   ble.print(gyro1.y(), 2);ble.print('\t');
   ble.print(" Z: ");
   ble.print(gyro1.z(), 2);
  ble.println();
 delay(200);
   //delay(BNO055_SAMPLERATE_DELAY_MS);
   if(acelerometer.x()>abs(maxAccel)){
   // maxAccel = acelerometer.x();
   }
  else if(acelerometer.y()>abs(maxAccel)){
    maxAccel =acelerometer.y();
  }
  else if(acelerometer.z()>abs(maxAccel)){
    maxAccel = acelerometer.z();
  }
  
 ble.print("Max Accel. ");ble.print(maxAccel);ble.println();
//delay(BNO055_SAMPLERATE_DELAY_MS);
  if(euler.x()>abs(maxTilt)){
    maxTilt = euler.x();
   }
  else if(euler.y()>abs(maxTilt)){
   // maxTilt =euler.y();
  }
  else if(euler.z()>abs(maxTilt)){
   // maxTilt = euler.z();
  }
  ble.print("Max Tilt ");ble.print(maxTilt);ble.println();
  ble.println();
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  delay(2000);
  // Check for user input
  //char n, inputs[BUFSIZE+1];
 // //ble.print("Test for work");
  //ble.print(Serial.available());
 /* if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
    
  }*/

  // Echo received data
 /* while ( ble.available() )
  {
    int c = ble.read();
  //  Serial.print("Echo received data as Hex?");
    //Serial.print((char)c);

    // Hex output too, helps w/debugging!
    //Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
   // Serial.print(c, DEC);//HEX
    //Serial.print("] ");
  }*/
  /* Wait for new data to arrive */
 // uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //if (len == 0) return;

  /* Got a packet! */
   //printHex(packetbuffer, len);//printHex

  
  
  // Accelerometer
  //ble.print("This is supposed to start printing accelerometer data");
  //if (packetbuffer[1] == 'A') {
  //  bno.getEvent(&event);
  /*  Serial.print(packetbuffer[1]);
    float x1, y2, z3;
    x1 = parsefloat(packetbuffer+2);//2
    y2 = parsefloat(packetbuffer+6);//6
    z3 = parsefloat(packetbuffer+10);//10
   // Serial.print("this is working aceel data--------");
    //Serial.print(accel);
  //  Serial.print('\t');
  //  Serial.print("------------Accel\t");
   ble.print("X accel  ");   ble.print(acelerometer.x(),4); ble.print('\t');
   ble.print("Y accel  "); ble.print(acelerometer.y(),4); ble.print('\t');
   ble.print("Z accel");ble.print(acelerometer.z(),4);ble.println(); //Serial.print(z);
     //ble.print(event.orientation.x, 4);
    ble.println();
   */
   
}

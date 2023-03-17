//////////////////////////////////////////////////////////////////////////////////////////
//
//    Demo code for the FDC1004 capacitance sensor breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    This example measures raw capacitance across CHANNEL0 and Gnd and
//    prints on serial terminal
//
//    Arduino connections:
//
//    Arduino   FDC1004 board
//    -------   -------------
//    5V - Vin
//   GND - GND
//    A4 - SDA
//    A5 - SCL
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/protocentral/ProtoCentral_fdc1004_breakout
/////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "Protocentral_FDC1004.h"

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"


#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL_A 0                          // channel to be read
#define CHANNEL_B 3                          // reference channel
#define MEASURMENT 0                       // measurment channel
#define UPPER_GAIN 1
#define LOWER_GAIN 0000

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int capdac = 0;
float offset_cal = 0;   // in pF
// char result[100];

// weight conversion 
float delta_change = 0.03; 
float initial_volume = 500.0;
float initial_cap = 5.36;

// for debugging
float initial_cap_val = 0;  // in pF
int initial_ms = 8000;
float end_cap_val;
float rate;   // pF/Hr
int debug_begin_delay = 8000;

FDC1004 FDC;

uint32_t update_timestamp = 0;

void setup()
{
  Wire.begin();        //i2c begin
  
  // Sets up the HW an the BLE module
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
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
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  delay(4000);
}

void loop()
{
  
  FDC.configureMeasurementDiffernetial(MEASURMENT, CHANNEL_A, CHANNEL_B);
  FDC.configureOffsetCalibration(MEASURMENT, offset_cal);
  FDC.configureGainCalibration(MEASURMENT, UPPER_GAIN, LOWER_GAIN);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);

  //wait for completion
  delay(15);
  uint16_t value[2];
  if (! FDC.readMeasurement(MEASURMENT, value))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    float cap_readout = (float)capacitance/1000;

    end_cap_val = cap_readout;
    rate = (end_cap_val - initial_cap_val)/((millis()-initial_ms)/1000); // pF per second
    rate = rate * 60 * 60;

    if( millis() - update_timestamp > 1000){
      ble.print(millis());
      ble.print(", ");
      ble.print((((float)capacitance/1000)),4);   // in pF
      ble.print(", ");
      ble.print(offset_cal);
      ble.print(", ");
      ble.println(capdac);
//      ble.print(", ");
//      ble.println(rate, 6);  // pf per hour

      update_timestamp = millis();
    }



  //   if (msb > UPPER_BOUND)               // adjust capdac accordingly
  // {
  //     if (capdac < FDC1004_CAPDAC_MAX)
  //   capdac++;
  //   }
  // else if (msb < LOWER_BOUND)
  // {
  //     if (capdac > 0)
  //   capdac--;
  //   }

  } else { 
    Serial.println("not ready");
    delay(1000);
  }
//  delay(1000);
}

/**************BNO55************************************* 
 This driver reads raw data from the BNO055

   Connections

   
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

// Bluetooth libraries
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

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
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// define the pins used
#define VS1053_RX  2 // This is the pin that connects to the RX pin on VS1053

#define VS1053_RESET 9 // This is the pin that connects to the RESET pin on VS1053
// If you have the Music Maker shield, you don't need to connect the RESET pin!

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79
#define instrument 5

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

SoftwareSerial VS1053_MIDI(0, 2); // TX only, do not use the 'rx' side
// on a Mega/Leonardo you may have to change the pin to one that 
// software serial support uses OR use a hardware serial port!

int analogPin = A3;
int breath = 0;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
    //**********bno55***********************
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);

  //**********MIDI*********************
  Serial.println("VS1053 MIDI test");
  
  VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate'
  
  pinMode(VS1053_RESET, OUTPUT);
  digitalWrite(VS1053_RESET, LOW);
  delay(10);
  digitalWrite(VS1053_RESET, HIGH);
  delay(10);
  
  midiSetChannelBank(0, VS1053_BANK_MELODY);
  midiSetInstrument(0, instrument); //INSTRUMENT
  midiSetChannelVolume(0, 127);

  //**********Bluetooth*********************
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

//  ble.print("AT+GAPDEVNAME=GROUP10");
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

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}
bool first = 1;

//----------------------------------------------------------------------------------------
void loop(void){
  delay(100);
  breath = analogRead(analogPin);
  Serial.print("Breath: ");
  Serial.println(breath);

  int midinote=0;
  String notestr;
  digitalWrite(VS1053_RESET, HIGH);
  //initialize bno55 data variables
  sensors_event_t event;
  bno.getEvent(&event);

  //notes mapping
  double orientx = event.orientation.x;
  double orienty = event.orientation.y;
  double orientz = event.orientation.z;
  midinote = maptonote(orientx, orienty, orientz);
  notestr = numtonote(midinote);
  
  Serial.print("\torientX: ");
  Serial.println(orientx);
  Serial.print("\torientY: ");
  Serial.println(orienty);
  Serial.print("\torientZ: ");
  Serial.println(orientz);

  if (breath > 275) {
     midiNoteOn(0, midinote, maptovolume(breath));
     ble.print("AT+BLEUARTTX=");
     ble.println(notestr);
     while (breath > 275) {
        midiNoteOff(0, 0, 0);
        breath = analogRead(analogPin);
     }
  }
  else if (breath > 260 && breath < 275) {
    midiNoteOff(0, midinote, 0);
    Serial.println("no volume");
  }
}

void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_PROGRAM | chan);  
  VS1053_MIDI.write(inst);
}


void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write(MIDI_CHAN_VOLUME);
  VS1053_MIDI.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write((uint8_t)MIDI_CHAN_BANK);
  VS1053_MIDI.write(bank);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) { // vel is the volume
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_ON | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

bool inrange(double x, double x1, double x2) {
  if (x>=x1 && x<x2) {
    return true;
  }
  else {
    return false;
    }
}

int maptovolume(double breath) {
  if (breath>=275 && breath <285) {
    return 80;
  }
  else if (breath>=285 && breath<295) {
    return 100;
  }
  else if (breath >=295) {
    return 127;
  }
}

int maptonote(double orientx, double orienty, double orientz) {
  int midinote;
  //bottom
  if (inrange(orienty, 10, 25) && inrange(orientz, 120, 140)) {
    midinote = 60;
  }
  //bottom up
  else if (inrange(orienty, 10, 25) && inrange(orientz, 140, 160)) {
    midinote = 62;
  }
  //middle up
  else if (inrange(orienty, 10, 25) && (inrange(orientz, 160, 180)||(inrange(orientz, -180, -165)))) {
    midinote = 64;
  }
  //left up
  else if (inrange(orienty, -10, 10) && (inrange(orientz, 160, 180)||inrange(orientz, -180, -165))) {
    midinote = 65;
  }
  //further left up
  else if (inrange(orienty, -35, -10) && (inrange(orientz, 160, 180)||inrange(orientz, -180, -165))) {
    midinote = 67;
  }
  //right up
  else if (inrange(orienty, 35, 50) && (inrange(orientz, 160, 180)||inrange(orientz, -180, -160))) {
    midinote = 69;
  }
  //further right up
  else if (inrange(orienty, 50, 65) && (inrange(orientz, 160, 180)||inrange(orientz, -180, -165))) {
    midinote = 71;
  }
  else {
    Serial.println("out of range");
    midinote = 40;
  }
  return midinote;
}

String numtonote(int num) {
  if (num==60) {
    return "C";
  }
  else if (num==61) {
    return "Cs";
  }
  else if (num==62) {
    return "D";
  }
  else if (num==63) {
    return "Ds";
  }
  else if (num==64) {
    return "E";
  }
  else if (num==65) {
    return "F";
  }
  else if (num==66) {
    return "Fs";
  }
  else if (num==67) {
    return "G";
  }
  else if (num==68) {
    return "Gs";
  }
  else if (num==69) {
    return "A";
  }
  else if (num==70) {
    return "Bb";
  }
  else if (num==71) {
    return "B";
  }
  else if (num==72) {
    return "C1";
  }
  else if (num==73) {
    return "Cs1";
  }
  else if (num==74) {
    return "D1";
  }
  else if (num==75) {
    return "Ds1";
  }
  else if (num==76) {
    return "E1";
  }
  else {
    return "invalid";
  }
}


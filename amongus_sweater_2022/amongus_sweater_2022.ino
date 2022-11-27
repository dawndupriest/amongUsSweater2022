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


/* Dawn's Ugly Christmas Sweater 2022
   Wiring:
   strand of neopixels (40 pixels): pin 12
   Neopixel 8x8 matrix: pin 11
   Featherwing Music Maker w amp connected 
   battery with switch
*/

/* music player headers */
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

// These are the pins used
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)

// Feather ESP8266
#if defined(ESP8266)
  #define VS1053_CS      16     // VS1053 chip select pin (output)
  #define VS1053_DCS     15     // VS1053 Data/command select pin (output)
  #define CARDCS          2     // Card chip select pin
  #define VS1053_DREQ     0     // VS1053 Data request, ideally an Interrupt pin

// Feather ESP32
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  #define VS1053_CS      32     // VS1053 chip select pin (output)
  #define VS1053_DCS     33     // VS1053 Data/command select pin (output)
  #define CARDCS         14     // Card chip select pin
  #define VS1053_DREQ    15     // VS1053 Data request, ideally an Interrupt pin

// Feather Teensy3
#elif defined(TEENSYDUINO)
  #define VS1053_CS       3     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          8     // Card chip select pin
  #define VS1053_DREQ     4     // VS1053 Data request, ideally an Interrupt pin

// WICED feather
#elif defined(ARDUINO_STM32_FEATHER)
  #define VS1053_CS       PC7     // VS1053 chip select pin (output)
  #define VS1053_DCS      PB4     // VS1053 Data/command select pin (output)
  #define CARDCS          PC5     // Card chip select pin
  #define VS1053_DREQ     PA15    // VS1053 Data request, ideally an Interrupt pin

#elif defined(ARDUINO_NRF52832_FEATHER )
  #define VS1053_CS       30     // VS1053 chip select pin (output)
  #define VS1053_DCS      11     // VS1053 Data/command select pin (output)
  #define CARDCS          27     // Card chip select pin
  #define VS1053_DREQ     31     // VS1053 Data request, ideally an Interrupt pin

// Feather M4, M0, 328, ESP32S2, nRF52840 or 32u4
#else
  #define VS1053_CS       6     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
  #define VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin

#endif






#include <Arduino.h>
#include <SPI.h>

#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif
#include <Adafruit_NeoPixel.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


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
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     1

    #define PIN                     11
    #define MATRIXPIN               12
    #define NUMPIXELS               40
    #define BPM                     118
    // MS PER BEAT is 60000 / BPM
    #define MSPERBEAT               508
/*=========================================================================*/

#define BLE_DISABLE  digitalWrite(BLUEFRUIT_SPI_CS,HIGH);
#define BLE_ENABLE   digitalWrite(BLUEFRUIT_SPI_CS,LOW);


Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN);
/*
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, MATRIXPIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);
*/  

// music player
  Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

uint32_t redcolor = pixel.Color(80,0,0);
uint32_t bluecolor = pixel.Color(0,0,80);
uint32_t greencolor = pixel.Color(0,60,0);
uint32_t yellowcolor = pixel.Color(100,100,0);
uint32_t browncolor = pixel.Color(25,6,0);
uint32_t cyancolor = pixel.Color(0,120,120);
uint32_t orangecolor = pixel.Color(120,50,0);
uint32_t limecolor = pixel.Color(20,82,0);
uint32_t pinkcolor = pixel.Color(144,45,114);
uint32_t whitecolor = pixel.Color(120,120,120);
uint32_t blackcolor = pixel.Color(8, 0, 15);
uint32_t purplecolor = pixel.Color(40,0,80);
uint32_t offcolor = pixel.Color(0,0,0);

uint32_t amonguscolors[] = {yellowcolor, bluecolor, browncolor, cyancolor, orangecolor, limecolor, pinkcolor, blackcolor, greencolor, purplecolor, whitecolor, redcolor};
uint8_t amongusnumbers[] = {38, 36, 34, 28, 30, 32, 25, 23, 21, 7, 11, 15};
uint8_t normalpixels[] = {39, 35, 37, 33, 27, 29, 31, 24, 22, 4, 3, 2};
uint8_t allpixels[] = {39, 35, 34, 36, 38, 37, 33, 32, 30, 28, 27, 29, 31, 21, 23, 25, 24, 22, 15, 11, 7, 4, 3, 2};

//char *amongusnames[] = {"Yellow","Blue","Brown","Cyan","Orange","Lime","Pink","Black","Green","Purple","White","Red"};
uint8_t amongusmatrix[4][3] ={{38,36,34},{28,30,32},{25,23,21},{7,11,15}};
//char *amongusnamematrix[4][3] = {{"Yellow","Blue","Brown"},{"Cyan","Orange","Lime"},{"Pink","Black","Green"},{"Purple","White","Red"}};


int musicenabled = 1;


int gamemode = 0;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

// returns true if an element is in the array twice. Returns false otherwise.
bool searchArray24Twice(int s, int sarray[])
{
  int j;
  int foundcount = 0;
  for (j=0; j<24; j++)
  {
    if(sarray[j] == s)
      foundcount++;
  }
  if(foundcount >= 2)
    return true;
  else
    return false;
}
/*
bool searchArray12(int s, int sarray[])
{
  int j;
  for (j=0; j<12; j++)
  {
    if(sarray[j] == s)
      return true;
  }
  return false;
}


void fillRandomArray12(int myarray[])
{
  
  int r = random(0,12);
  bool found = false;
  for(int n = 0; n < 12; n++)
  {
    while(searchArray12(r, myarray))
      r = random(0,12);
    myarray[n] = r;
  }
  
} 
*/


void fillRandomArray24(int myarray[])
{
  
  int r = random(0,12);
  bool found = false;
  for(int n = 0; n < 24; n++)
  {
    while(searchArray24Twice(r, myarray))
      r = random(0,12);
    myarray[n] = r;
    r = random(0,12);
  }
}  

void setup(void)
{
  randomSeed(analogRead(A1));
  
  pixel.begin(); // This initializes the NeoPixel library.
  
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  pixel.show();

  Serial.begin(115200);

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

  BLE_DISABLE;
  musicPlayer.begin();
  //musicPlayer.sineTest(0x44, 500);
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    musicenabled = 0; // can't play music
  }
  else
  {
    Serial.println("SD OK!");
    musicPlayer.setVolume(10,10);
  
#if defined(__AVR_ATmega32U4__) 
  // Timer interrupts are not suggested, better to use DREQ interrupt!
  // but we don't have them on the 32u4 feather...
    musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int
#else
  // If DREQ is on an interrupt pin we can do background
  // audio playing
    musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
#endif
  }

  BLE_ENABLE;
  

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  //ble.info();

  Serial.println(F("Ready!"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  //Serial.println(F("***********************"));

  
  

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  // if no data and not playing a game then loop again
  if (len == 0 && gamemode == 0) return;
  // if playing game 1
  if (gamemode == 1)
  {

    BLE_DISABLE;
  musicPlayer.startPlayingFile("/amongus.mp3");
  clearPixels();
  // 24 elements in array. Cycle through list of crewmates twice.
    int rarray[] = {13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13};
    
    fillRandomArray24(rarray);
    
    uint8_t i;
    for(i = 0; i < 23; i++)
    {
      pixel.setPixelColor(amongusnumbers[rarray[i]],amonguscolors[rarray[i]]); 
      pixel.show();
      delay(200);
      //Serial.print(amongusnumbers[rarray[i]]);
      //Serial.print(", ");
      pixel.setPixelColor(amongusnumbers[rarray[i]],offcolor);
      pixel.show();
      delay(50);     
    }
    //Serial.println(":");
    
    // the impostor is the last element in the second randomized array
    //String impostorname = amongusnames[rarray[i]];
    int impostornumber = amongusnumbers[rarray[i]];
    uint32_t impostorcolor = amonguscolors[rarray[i]];
    int impostorxcoord;
    int impostorycoord;
    //Serial.print(F("The Impostor Is: "));
    //Serial.println(impostornumber);
    
    for(int a = 0; a < 4; a++)
    {
      for(int b = 0; b < 3; b++)
      { 
        if(amongusmatrix[a][b] == impostornumber)
        {
          impostorycoord=a;
          impostorxcoord=b;
          Serial.print("*");
        }            
        Serial.print(amongusmatrix[a][b]);   
        Serial.print(",");
      }
      Serial.println(";");
    }

    musicPlayer.stopPlaying();
    delay(500);
    BLE_ENABLE;    
    // now try to guess
    int numtries = 0;

    int ypos = random(0,4);
    int xpos = random(0,3);
    pixel.setPixelColor(amongusmatrix[ypos][xpos],whitecolor); 
    pixel.show();
    while(gamemode == 1)
    {
      len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
      while(len == 0)
      {
        len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
      }
      if(packetbuffer[1] == 'B')
      {
        int x;
        // copied and pasted from the example. I guess "buttnum" is the variable name.
        uint8_t buttnum = packetbuffer[2] - '0';
        boolean pressed = packetbuffer[3] - '0';
        Serial.print (F("Button ")); Serial.print(buttnum);
        if (pressed) {
          Serial.println(" pressed");
          // up arrow
          if(buttnum == 5 || buttnum == 6 || buttnum == 7 || buttnum == 8)
          {
            pixel.setPixelColor(amongusmatrix[ypos][xpos],offcolor); 
            pixel.show();
            if(buttnum == 5)
            {            
            ypos = ypos - 1;
            if(ypos < 0)
              ypos = 3;                        
            }
          // down arrow
            else if(buttnum == 6)
            {
              ypos = ypos + 1;
              if(ypos > 3)
               ypos = 0;                        
            }
          // left arrow
            else if(buttnum == 7)
            {
              xpos = xpos - 1;
              if(xpos < 0)
                xpos = 2;                        
            }
          // left arrow
            else if(buttnum == 8)
            {
              xpos = xpos + 1;
              if(xpos > 2)
                xpos = 0;                        
            }
          pixel.setPixelColor(amongusmatrix[ypos][xpos],whitecolor); 
          pixel.show();
          }
          // check to see if you won
          if(buttnum == 1)
          {
            BLE_DISABLE;
            if(impostorycoord == ypos && impostorxcoord == xpos)
            {
              // YOU WIN!
              musicPlayer.startPlayingFile("/youwin.mp3");
              for (int q = 0; q < 12; q++)
              {
                pixel.setPixelColor(amongusnumbers[q],amonguscolors[q]);
                pixel.show();
                delay(150);
              }
              delay(4000);
              gamemode = 0;
              clearPixels();
              BLE_ENABLE;
              return;               

            }
            else
            {
              // incorrect! Make an "x"
              numtries = numtries + 1;  
              pixel.setPixelColor(34, redcolor);
              pixel.setPixelColor(30, redcolor);
              pixel.setPixelColor(25, redcolor);
              pixel.setPixelColor(38, redcolor);
              pixel.setPixelColor(21, redcolor);
              pixel.show();
              
                if(numtries >=3)
                {
                  // if you really, really lost, show who the imposter was.
                    pixel.setPixelColor(impostornumber,impostorcolor);
                    pixel.show();
                    musicPlayer.playFullFile("/youlose.mp3");
                       
                }
                else
                {
                  musicPlayer.playFullFile("/wrong.mp3");
                }
                BLE_ENABLE;
              
              pixel.clear();
              if(numtries >=3)
              {
                gamemode = 0;       
              }     
              else
              {
                pixel.setPixelColor(amongusmatrix[ypos][xpos],whitecolor);  
              }              
              pixel.show();
                     
            }            
            
            
          }
          if(buttnum == 4)  // button 4 cancels out of the game.
          {
            gamemode = 0;
            clearPixels();
            
            
          }      
        }
      }              
    }
    gamemode = 0;
    return;
  
    }  

  /* Got a packet! */
  // printHex(packetbuffer, len);

// if you're not playing the game, you can do one of these options. Option 1: set the pixels to a color from the color picker.
  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

    for(uint8_t i=0; i<24; i++) {
      pixel.setPixelColor(allpixels[i], pixel.Color(red,green,blue));
    }
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  else if(packetbuffer[1] == 'B')
  {
    // copied and pasted from the example. I guess "buttnum" is the variable name.
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print (F("Button ")); Serial.print(buttnum);
    if (pressed) {
      Serial.println(F(" pressed"));
      // button 1: play the game
      if(buttnum == 1)
      {
          gamemode = 1;
                  
      }
      // button 2: play the song with lightshow
      if(buttnum == 2)
      {
        BLE_DISABLE;
        musicPlayer.startPlayingFile("show2.mp3");
        delay(MSPERBEAT*1);
        theaterChaseDuration(redcolor, 50, MSPERBEAT * 2, false);
        theaterChaseDuration(whitecolor, 50, MSPERBEAT * 1, true);
        dontYou(MSPERBEAT*4);
        theaterChaseDuration(cyancolor, 50, (int)(MSPERBEAT * 2), false);
        theaterChaseDuration(whitecolor, 50, MSPERBEAT * 1, true);
        dontYou(MSPERBEAT*4);
        theaterChaseRainbow(50, MSPERBEAT*3);
        colorWipe(pinkcolor, MSPERBEAT * 2);
        fadeInOut(whitecolor, MSPERBEAT * 3);
        colorWipe(bluecolor, MSPERBEAT * 2);
        colorWipe(yellowcolor, MSPERBEAT * 2);
        dontYou(MSPERBEAT*2);
        fadeInOut(purplecolor, MSPERBEAT * 1);
        clearPixels();
        BLE_ENABLE;
      }
      // button 4: cancel out of whatever
      if(buttnum == 4)
      {
        gamemode = 0;
        clearPixels();
      }

    }   
        
  }

}

void colorWipe(uint32_t color, int duration) {
  int totalpixels = 24;
  int eltime = 0;
  int waittime = duration / totalpixels;
  for(int i=0; i<totalpixels && eltime < duration; i++) { // For each pixel in strip...
    pixel.setPixelColor(allpixels[i], color);         //  Set pixel's color (in RAM)
    pixel.show();                          //  Update strip to match
    delay(waittime);                           //  Pause for a moment
    eltime = eltime + waittime;
  }
  clearPixels();
}
void theaterChaseDuration(uint32_t color, int wait, int duration, bool impostor) {
  int eltime = 0;
  int offset = 0;
  int totalpixels = 24;
  if(impostor)
  {
    offset = 21;
    totalpixels = 3;
  }
  while (eltime < duration)
  {
    for(int b=0; b<3 && eltime < duration; b++) 
    { //  'b' counts from 0 to 2...
      pixel.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b + offset; c<totalpixels + offset; c += 3) {
        pixel.setPixelColor(allpixels[c], color); // Set pixel 'c' to value 'color'
      }
      pixel.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
      eltime += (int)(wait); // only 90% of the delay because this is too long
    }
  }
  clearPixels();
}


void dontYou(uint16_t mseach)
{
  for(int i = 0; i < 12; i = i + 2)
  {
    pixel.setPixelColor(amongusnumbers[i],amonguscolors[i]);
  }
  pixel.show();
  delay(mseach/2);
  pixel.clear();
  pixel.show();
  for(int i = 1; i < 12; i = i + 2)
  {
    pixel.setPixelColor(amongusnumbers[i],amonguscolors[i]);
  }
  pixel.show();
  delay(mseach/2);
  clearPixels();
}

void fadeInOut(uint32_t color, int duration)
{
    int eltime = 0;
    int brightchange = 10;
    int bright = 0;
    while(eltime < duration)
    {
      for(int i = 0; i < 12; i++)
      {
        pixel.setPixelColor(normalpixels[i],color);
        pixel.setBrightness(bright);
      }
      pixel.show();
      delay((int)(duration / 2 / 10));
      eltime = eltime + (int)(duration / 2 / 10);
      bright = bright + brightchange;
      if(eltime > duration / 2)
          brightchange = brightchange * -1;
    }
    clearPixels();
}

void theaterChaseRainbow(int wait, int duration) {

  int eltime = 0;
  int offset = 0;
  int totalpixels = 24;
  int pixelcolorindex = 0;  
  while (eltime < duration)
  {
    for(int b=0; b<3 && eltime < duration; b++) 
    {
      pixel.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<totalpixels; c += 3) {
        uint32_t color = amonguscolors[pixelcolorindex];
        pixel.setPixelColor(allpixels[c], color); // Set pixel 'c' to value 'color'
        pixelcolorindex = pixelcolorindex + 1;
        if(pixelcolorindex >= 12)
          pixelcolorindex = 0;
      }
      pixel.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      eltime += wait;
    }
  }
  clearPixels();
}

void clearPixels()
{
  pixel.clear();
  pixel.show();
}

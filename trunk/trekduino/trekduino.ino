/* 
Customized Star Trek Enterprise model with ambiental white leds, red led for red alarm, RGB led for engines,
red and green leds for position lights, red led for deflector, real laser for laser ;-) , 
red led for photon torpedo, voice recognice with easyVR, sounds, temperature  and humidity sensor, 
TFT display to show images and messages, PIR sensor to detect movements.
Build with Arduino Mega.
Autor: Antonio Garcia Figueras

Pines físicos del cableado de la nave:
De izq a derecha. izq marcarcado con pintura marrón.
Bloque 1 (partido en dos partes):
* Leds verdes y rojos de posición (3,3 volt):
  1- azul (puerto dig 11).
  2- amarillo (tierra)
* Leds blancos iluminación plato (5 volt):
  3- rojo (puerto dig 16)
  4- blanco (tierra)
 * Leds blancos barquilla (5 volt):
   5- rojo (puerto dig 17)
   6- negro (tierra)
 * Leds rojos alerta roja (5 volt):
   7- rojo (puerto dig 18)
   8- negro (tierra)
 * Led torpedo fotones (5 volt):
   09- azul (puerto dig 5)
   10- negro (tierra)
 * Laser (3,3 volt):
   09- azul (puerto dig 6)
   10- amarillo (tierra)
 * Led rojo antena deflector (3,3 volt):
   09- negro (tierra)
   10- rojo (puerto dig 10)
   
Bloque 2. Los led RGB de los motores derecho e izquierdo comparten los puertos digitales:
* Led RGB motor derecho:
  1- rojo (color verde) (puerto dig 3)
  2- negro (color azul) (puerto dig 2)
  3- rojo (ánodo) (5 volt.)
  4- negro (color rojo) (puerto dig 7)
* Led RGB motor izquierdo:
  1- rojo (color verde) (puerto dig 3)
  2- negro (color azul) (puerto dig 2)
  3- rojo (ánodo) (5 volt.)
  4- negro (color rojo) (puerto dig 7)
*/

// Inicio declaraciones para tft ******************
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <SD.h>

// TFT display and SD card will share the hardware SPI interface.
// Hardware SPI pins are specific to the Arduino board type and
// cannot be remapped to alternate pins.  For Arduino Uno,
// Duemilanove, etc., pin 11 = MOSI, pin 12 = MISO, pin 13 = SCK.
#define SD_CS    4  // Chip select line for SD card
//#define TFT_CS  10  // Chip select line for TFT display
#define TFT_CS  53  // Chip select line for TFT display
#define TFT_DC   9  // Data/command line for TFT
#define TFT_RST  8  // Reset line for TFT (or connect to +5V)

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
// Fin declaraciones para TFT ******************

// Inicio declaraciones para EasyVr *************

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #include "SoftwareSerial.h"
  SoftwareSerial port(12,13);
#else // Arduino 0022 - use modified NewSoftSerial
  #include "WProgram.h"
  #include "NewSoftSerial.h"
  NewSoftSerial port(12,13);
#endif

#include "EasyVR.h"
EasyVR easyvr(port);

//Groups and Commands
enum Groups
{
  GROUP_2  = 2,
  GROUP_3  = 3,
};

enum Group2 
{
  G2_ORDENADOR = 0,
};

enum Group3 
{
  G3_ALERTA_ROJA = 0,
  G3_LEVANTA_ESCUDOS = 1,
  G3_DISPARA_FASERS = 2,
  G3_DISPARA_TORPEDO_FOTO = 3,
};


EasyVRBridge bridge;

int8_t group, idx;
// Fin declaraciones para EasyVR **************

//Declaraciones para DHT11 Arduino Compatible Digital Temperature Humidity Sensor Module
// The pins were not the same as documentation on internet, being left pin (marked 's') signal, medium Vcc and right (marked '1') GND. In addition, the pull up resistor which is shown in the datasheets is already installed (10Koms resistor) 
// https://www.virtuabotix.com/
#include <dht11.h>
dht11 DHT11;
int dht11Pin = 15;
// Fin declaraciones DHT11

// Green and red blinking sidelights 
long sidelightsPreviousMillis = 0;
unsigned long sidelightscurrentMillis;
boolean sidelightsState;
int sidelightsPin = 11;
long sidelightsInterval = 3000;

int plateWhiteLightsPin = 16;
int bodyWhiteLightsPin = 17;
int redAlertLightsPin = 18;
int fotonTorpedoPin = 5;
int faserPin = 6;
int deflectorPin = 10;

// Engines
int redPin = 7;
int greenPin = 3;
int bluePin = 2;
long enginePreviousMillis = 0;
unsigned long engineCurrentMillis;
boolean engineState;
long engineInterval = 700;
int redColour;

// PIR sensor
int inputPin = 19;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status

// Button
const int buttonPin = 20;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
long buttonInitialMillis = 0;
unsigned long buttonCurrentMillis;
long buttonInterval = 20000; 

//// Display
long displayInitialMillis = 0;
unsigned long displayCurrentMillis;
long temperatureInterval = 3000;
long imageInterval = 15000; // imageInterval must be greater than temperatureInterval
int displayState=0; // 0 = Temperature, 1 = Image

// Motion
long motionInitialMillis = 0;
unsigned long motionCurrentMillis;
long motionInterval = 60000; 
boolean motionState = true;

void setup()
{
setupEasyVr();
setupTft();
pinMode(plateWhiteLightsPin, OUTPUT);
pinMode(bodyWhiteLightsPin, OUTPUT);
pinMode(redAlertLightsPin, OUTPUT);
pinMode(fotonTorpedoPin, OUTPUT);
pinMode(faserPin, OUTPUT);
pinMode(deflectorPin, OUTPUT);
pinMode(redPin, OUTPUT);
pinMode(greenPin, OUTPUT);
pinMode(bluePin, OUTPUT);

//Engines lights
analogWrite(redPin, 255);
analogWrite(greenPin, 255);
analogWrite(bluePin, 255); 

plateWhiteLights(true);
bodyWhiteLights(true);
deflector(true);

DHT11.attach(dht11Pin); 

pinMode(buttonPin, INPUT);
}


void loop()
{
  if (isMotion()){
//    Serial.println("====================================Motion detected");
    motionState=true;
    motionInitialMillis = millis();
    do{
//      Serial.println("*************Engage");
      engage();
      motionCurrentMillis = millis();
    }while (motionCurrentMillis - motionInitialMillis < motionInterval);    
  } else {
    if (motionState){
//      Serial.println("*************FullStop");
      fullStop();
      motionState=false;
    }
  }
}


void engage(){
    plateWhiteLights(true);
    bodyWhiteLights(true);
    deflector(true);      
    button();
    sidelights();
    engine();
    display();
}


void fullStop(){
    plateWhiteLights(false);
    bodyWhiteLights(false);
    deflector(false);  
  
    turnOffSidelights();
  
    //Engines lights OFF
    analogWrite(redPin, 255);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255); 

    tft.fillScreen(ST7735_BLACK);
}

void setupEasyVr()
{
  // bridge mode?
  if (bridge.check())
  {
    cli();
    bridge.loop(0, 1, 12, 13);
  }
  // run normally
  Serial.begin(9600);
  port.begin(9600);

  if (!easyvr.detect())
  {
    Serial.println("EasyVR not detected!");
    for (;;);
  }

  easyvr.setPinOutput(EasyVR::IO1, LOW);
  Serial.println("EasyVR detected!");
  easyvr.setTimeout(5);
  easyvr.setLanguage(4);

//  group = EasyVR::TRIGGER; //<-- start group (customize)
  group = GROUP_2;  
}


void setupTft(){
   //Para TFT
    // If your TFT's plastic wrap has a Red Tab, use the following:
//  tft.initR(INITR_REDTAB);   // initialize a ST7735R chip, red tab
  // If your TFT's plastic wrap has a Green Tab, use the following:
  tft.initR(INITR_GREENTAB); // initialize a ST7735R chip, green tab

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("OK!");

//  bmpDraw("red1.bmp", 0, 0);
//  delay(3000);
//  bmpDraw("green1.bmp", 0, 0);
//  delay(3000);
  bmpDraw("panel.bmp", 0, 0);
//  delay(3000);
//  bmpDraw("red2.bmp", 0, 0);
//  delay(3000);
//  bmpDraw("trek1.bmp", 0, 0);
//  delay(3000);  
//  bmpDraw("UFOPlogo.bmp", 0, 0);
//  delay(3000);  
//  bmpDraw("yellow2.bmp", 0, 0);
//  delay(3000);   
}



void sidelights(){
  sidelightscurrentMillis = millis();
  if(sidelightscurrentMillis - sidelightsPreviousMillis > sidelightsInterval){// timer hits limit
    if (sidelightsState){
      analogWrite(sidelightsPin, 180); //turn on to 180 because that leds work with 3,3volts. 255 would be to 5volts
    }else{
      turnOffSidelights();
    }
    sidelightsPreviousMillis = sidelightscurrentMillis;
    sidelightsState = !sidelightsState; //flip the sidelights
  }
}

void turnOffSidelights(){
  analogWrite(sidelightsPin, 0); //turn off
}

void plateWhiteLights(boolean state){
   if (state){
    digitalWrite(plateWhiteLightsPin, HIGH);
   }else{
    digitalWrite(plateWhiteLightsPin, LOW);
   }
}

void bodyWhiteLights(boolean state){
   if (state){
    digitalWrite(bodyWhiteLightsPin, HIGH);
   }else{
    digitalWrite(bodyWhiteLightsPin, LOW);
   }
}

void redAlertLights(boolean state){
   if (state){
    digitalWrite(redAlertLightsPin, HIGH);
   }else{
    digitalWrite(redAlertLightsPin, LOW);
   }
}


void fotonTorpedo(){
  for (int i=0; i<=20; i++){
   analogWrite(fotonTorpedoPin, i);
   delay(100);
  }
//  Serial.println("Estallido");
  analogWrite(fotonTorpedoPin, 255);
  delay(10);
  analogWrite(fotonTorpedoPin, 0);
  easyvr.playSound(4, EasyVR::VOL_FULL);
}

void faser(){
 analogWrite(faserPin, 180);
 easyvr.playSound(3, EasyVR::VOL_FULL);
 analogWrite(faserPin, 0);
}

void deflector(boolean state){
   if (state){
    analogWrite(deflectorPin, 180); // Max 3,3volt
   }else{
    analogWrite(deflectorPin, 0);
   }
}

void engine(){
  engineCurrentMillis = millis();
  if(engineCurrentMillis - enginePreviousMillis > engineInterval){// timer hits limit

    if (engineState){
      redColour = 170; //Hard
    }else{
      redColour = 220; //Soft
    }
    analogWrite(redPin, redColour);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255); 
    enginePreviousMillis = engineCurrentMillis;
    engineState = !engineState; //flip engines colour 
  }
}

void humidityAndTemperature()
{
  Serial.println("\n");

  int chk = DHT11.read();

  Serial.print("Read sensor DHT11: ");
  switch (chk)
  {
    case 0: Serial.println("OK"); break;
    case -1: Serial.println("Checksum error"); break;
    case -2: Serial.println("Time out error"); break;
    default: Serial.println("Unknown error"); break;
  }

//  Serial.print("Humidity (%): ");
//  Serial.println((float)DHT11.humidity, DEC);
//
//  Serial.print("Temperature (°C): ");
//  Serial.println((float)DHT11.temperature, DEC);
//
//  Serial.print("Dew Point (°C): ");
//  Serial.println(DHT11.dewPoint(), DEC);
//
//  Serial.print("Dew PointFast (°C): ");
//  Serial.println(DHT11.dewPointFast(), DEC);
  
  // large block of text
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(3);
  tft.setRotation(1);
  
  //Temperature
  tft.setCursor(5, 20);
  char temp[10];
  float temperature = (float)DHT11.temperature;
  dtostrf(temperature,1,2,temp);
  String labelTem = "T:";
  String textTem = labelTem + temp;
  tft.print(textTem);
  
  //Humidity
  tft.setCursor(5, 80);
  char hum[10];
  float humidity = (float)DHT11.humidity;
  dtostrf(humidity,1,2,hum);
  String labelHum = "H:";
  String textHum = labelHum + hum;  
  tft.print(textHum);
//  delay(5000);  
}


//Checking for movement
void PIR(){
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
//    digitalWrite(ledPin, HIGH);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
//    digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }  
}


boolean isMotion(){
  if (digitalRead(inputPin)==HIGH){
    return true;
  } else {
    return false;
  }
}


// Check button. If pressed wait a voice command for 1 minute
void button(){
//  Serial.println("En button");
  if (digitalRead(buttonPin) == LOW){
    //Button pressed
    Serial.println("Button pressed");
    easyvr.playSound(2, EasyVR::VOL_FULL);//Beep    
    tft.setRotation(0);
    bmpDraw("standby.bmp", 0, 0);
    buttonInitialMillis = millis();
    group = GROUP_2; //initialice voice command
    do{
      // waiting for voice command
      reconocimientoVoz();
      buttonCurrentMillis = millis();
    }while (buttonCurrentMillis - buttonInitialMillis < buttonInterval);
    Serial.println("Timeout button"); 
    motionInitialMillis = millis(); //so we continium awake  
  }
}


// Display control
void display(){
  displayCurrentMillis = millis();
  if ((displayCurrentMillis - displayInitialMillis) < temperatureInterval){
    if (displayState != 0){
      displayState=0;
      humidityAndTemperature();
    }
    return;
  }
  if((displayCurrentMillis - displayInitialMillis) > temperatureInterval and
      (displayCurrentMillis - displayInitialMillis) < imageInterval){
    if (displayState != 1){
      displayState=1;  
      tft.setRotation(0);
      bmpDraw("panel.bmp", 0, 0);
    }
    return;
  }
  if ((displayCurrentMillis < displayInitialMillis) or (displayCurrentMillis - displayInitialMillis) > imageInterval){
    displayInitialMillis = displayCurrentMillis;
  }
}

void redAlert(){
  easyvr.playSound(1, EasyVR::VOL_FULL); //Alert
  tft.setRotation(0);
  bmpDraw("red2.bmp", 0, 0);
  plateWhiteLights(false);
  redAlertLights(true);
  easyvr.playSound(5, EasyVR::VOL_FULL); //Levanta escudos
  delay(2000);
  faser();
  delay(2000);
  fotonTorpedo();
  delay(5000);
  easyvr.playSound(2, EasyVR::VOL_FULL);//Beep    
  bmpDraw("yellow2.bmp", 0, 0);
  delay(5000);
  easyvr.playSound(2, EasyVR::VOL_FULL);//Beep    
  bmpDraw("green1.bmp", 0, 0);
  easyvr.playSound(5, EasyVR::VOL_FULL); //Baja escudos
//  delay(2000);
  redAlertLights(false);
  engage();
  delay(1000);
}

void reconocimientoVoz(){
  easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)

  Serial.print("Say a command in Group ");
  Serial.println(group);
  easyvr.recognizeCommand(group);

  do
  {
    // can do some processing while waiting for a spoken command
    if (digitalRead(buttonPin) == LOW){
      //If button pressed again redAlert and don't wait for a spoken command
      Serial.println("Button pressed");
      easyvr.playSound(2, EasyVR::VOL_FULL);//Beep   
      redAlert();
      break;
    }
  }
  while (!easyvr.hasFinished());
  
  easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
  if (idx >= 0)
  {
    // built-in trigger (ROBOT)
    // group = GROUP_X; <-- jump to another group X
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // print debug message
    uint8_t train = 0;
    char name[32];
    Serial.print("Command: ");
    Serial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train))
    {
      Serial.print(" = ");
      Serial.println(name);
    }
    else
      Serial.println();
//    easyvr.playSound(0, EasyVR::VOL_FULL);
    // perform some action
    actionEasyVr();
  }
  else // errors or timeout
  {
    if (easyvr.isTimeout())
      Serial.println("Timed out, try again...");
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      Serial.print("Error ");
      Serial.println(err, HEX);
    }
  }
}



void actionEasyVr()
{
    switch (group)
    {
    case GROUP_2:
      switch (idx)
      {
      case G2_ORDENADOR:
        // write your action code here
        easyvr.playSound(6, EasyVR::VOL_FULL);
        group = GROUP_3; //to jump to group 3 for composite commands
        break;
      }
      break;
    case GROUP_3:
      switch (idx)
      {
      case G3_ALERTA_ROJA:
//        easyvr.playSound(2, EasyVR::VOL_FULL);  //input sucessful
//        easyvr.playSound(1, EasyVR::VOL_FULL);
        redAlert();
        group = GROUP_2; //to jump to group 2 to start again
        break;
      case G3_LEVANTA_ESCUDOS:
//        easyvr.playSound(2, EasyVR::VOL_FULL);  //input sucessful
        easyvr.playSound(5, EasyVR::VOL_FULL);
        group = GROUP_2; //to jump to group 2 to start again
        break;
      case G3_DISPARA_FASERS:
//        easyvr.playSound(2, EasyVR::VOL_FULL);  //input sucessful      
        easyvr.playSound(3, EasyVR::VOL_FULL);
        group = GROUP_2; //to jump to group 2 to start again
        break;
      case G3_DISPARA_TORPEDO_FOTO:
//        easyvr.playSound(2, EasyVR::VOL_FULL);  //input sucessful      
        easyvr.playSound(4, EasyVR::VOL_FULL);
        group = GROUP_2; //to jump to group 2 to start again
        break;
      }
      break;
    }
}

//Para TFT
// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

//  Serial.println();
//  Serial.print("Loading image '");
//  Serial.print(filename);
//  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print("File size: "); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
//    Serial.print("Image Offset: "); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print("Header size: "); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
//      Serial.print("Bit Depth: "); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
//        Serial.print("Image size: ");
//        Serial.print(bmpWidth);
//        Serial.print('x');
//        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
//        Serial.print("Loaded in ");
//        Serial.print(millis() - startTime);
//        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
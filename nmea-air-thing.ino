
/*
  NMEA Air Thing

  Provides a temperature, humidity, pressure (and with a BME690) air quality monitor
        with NMEA formatted XDR sentence broadcasting on UDP 10110
        by WiFi to a local network WiFi configurable local SSID.

  The circuit:
    I2C bus: BME690 (or SHT3X-DIS)tbd
             No display
    D5, D6   No rotary selector
    D7       No button

  Created 25 January 2022
  Updated 17 November 2022
  By Nicholas Taylor

  http://url/of/online/tutorial.cc
  https://stackr.ca

*/

/*

   So the idea is that you won't have to install many additional libraries.
   But the reality is there are some very complicated libraries out there.

   So, all my code is in this file.

*/

/*
   These three do really cool things with numbers which I want to explore.
*/
#include <konfig.h>
#include <matrix.h>
#include <ukf.h>

#include <FS.h> // (this needs to be first, or it all crashes and burn)

#include <LiquidCrystal_I2C.h> // Library for LCD
#include <BigFont02_I2C.h>

/*
   My preferred 16bit ADS because eventually we will want this.
*/
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adsCurrent;  /* Use this for the 16-bit version */
Adafruit_ADS1115 adsVoltage;

#include "SHTSensor.h"
SHTSensor sht;

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#include <ESP_EEPROM.h>

#include <ESP8266WiFi.h>
#include <ESP8266TrueRandom.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

#include <WiFiUdp.h>
#include <Wire.h>
#include <OneButton.h>

#include <RotaryEncoder.h>


// Example for ESP8266 NodeMCU with input signals on pin D5 and D6
#define PIN_IN1 D5
#define PIN_IN2 D6

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

bool consoleFlag = false;

byte uuidNumber[16]; // UUIDs in binary form are 16 bytes long
String uuidStr;
String nuuidStr;

//flag for saving data
bool shouldSaveConfig = false;

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

int lastMillis = micros();

IRAM_ATTR void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

// Set web server port number to 80
WiFiServer server(8080);

// Variable to store the HTTP request
String header;


IPAddress ipBroadCast(192, 168, 10, 255);

boolean displayFlag = false;

/* Bar graph display stuff

*/
int delt_t = 0;

static float G = 9.81;
unsigned char b, b2;
double a, a2;
unsigned int segment, segment2;
double perc = 100;
boolean logar;
byte bn;                 // bn BAR NUMBER every bar have to be numbered from 1 to 40
unsigned int xpos, ypos, blen;
double pkval[41];        // to store the last peak value for each bar
int    pkcyc[41][2];     // set the num. of printbar recall TO DO [1] and DONE [2] for each bar before the peak decays,

// if pkcyc[bn][1] == 0 no peaks wanted
// it's a workaround to avoid to waste time in delay functions (internal or external)
// that may interfere your application performances

// Bar characters definitions (8 maximum allowed)

byte blockHorizontal[8][8] =
{
  { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 }, // define characters for bar
  { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
  { 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },
  { 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E },
  { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 }, // define characters for peak
  { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
  { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 },
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 },
};

// Bar characters definitions (8 maximum allowed)
byte blockVertical[8][8] =
{
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F }, // define characters for bar
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
};

/*
   OneButton menu stuff
*/


bool editFlag = false;
bool selectFlag = false;
bool scrollFlag = false;
boolean firstLoopFlag = true;

int changeAmount = 0;
bool encoderChangeFlag = false;
int encoderChangeAmount = 0;

/*
   Implementation specific
*/

// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

bool shtFlag = true;

bool oneWireFlag = true;

bool adsFlag = true;


int adcNumBits = 32767;

int voltageMillis[4];
int currentMillis[4];

int voltageElapsedMillis[4];
int currentElapsedMillis[4];


int startTimeCurrentDisplay[4];
int startTimeVoltageDisplay[4];
int startTimeCoulombDisplay[4];

int elapsedTimeXDR[4];

float voltage[4];
float current[4];

float gainVoltage[4];
float biasVoltage[4];

float gainCurrent[4];
float biasCurrent[4];

float shuntMaxCurrent[4];
float shuntMaxVoltage[4];


int countAvailableUpdateFrequencies = 16;
float availableUpdateFrequencies[] = {0, 0.0000115741, 0.000034722222, 0.0000694444, 0.000277778, 0.001111111, 0.016666667, 0.0625, 0.125, 0.25, 0.5, 1, 2, 5, 10, 20};

char * availableUpdateFrequenciesLabels[] = {
  " OFF ",
  " 24hr",
  "  8hr",
  "  4hr",
  "  1hr",
  " 15m ",
  " 60s ",
  " 15s ",
  "7.5s ",
  "  4s ",
  "  2s ",
  "  1Hz",
  "  2Hz",
  "  5Hz",
  " 10Hz",
  " 20Hz"
};

/*
   Screen and menu related variables
*/

static int screenIndex = 0;
static int lastScreenIndex = 0;
static int maxScreenIndex = 25;

static int selectIndex = 0;
static int lastSelectIndex = 0;

static int editIndex = 0;
static int lastEditIndex = 0;
int screenChangeAmount = 0;
int selectChangeAmount = 0;
int editChangeAmount = 0;

static int scrollIndex = 0;
static int lastScrollIndex = 0;
int scrollChangeAmount = 0;
static int maxScrollIndex = 13;
static int scrollOffset = 0;

bool broadcastWiFiFlag = false;
bool broadcastNMEAFlag = true;
bool broadcastXDRFlag = true;

int startTimeNMEA = 0;
int startTimeXDR[4];

float broadcastXDRFrequency[4];

int startTimeSensorRead = micros();
float sensorReadFrequency = 1.0 / 20; // Read sensor every 20s.
float sensorSHTReadFrequency = 10.0; // 10Hz
float sensorOneWireReadFrequency = 10.0; //10 Hz

boolean sensorReadFlag = true;

int elapsedTimeSensorRead = 0;

//     if (elapsedTimeSensorRead > 1 / sensorReadFrequency[i] * 1e6) {

unsigned long endTime = 0;
unsigned long endSHTTime = 0;
unsigned long endOneWireTime = 0;

unsigned long startWebRequestTime = 0;
unsigned long runTimeWebRequestTime = 0;

OneButton button(D7, true, true);

int lastCount = 0;

unsigned int udpRemotePort = 10110;

int maxHistoryCurrent[4];
int windowHistoryCurrent[4];

int maxHistoryVoltage[4];
int windowHistoryVoltage[4];

int maxHistoryCoulomb[4];
int windowHistoryCoulomb[4];

const int maxHistoryCurrent0  = 200;
static int windowHistoryCurrent0 = 50;

const int maxHistoryWindow  = 200;

const int maxHistoryCurrent1  = 200;
static int windowHistoryCurrent1 = 50;

const int maxHistoryCurrent2  = 200;
static int windowHistoryCurrent2 = 50;

const int maxHistoryCurrent3  = 200;
static int windowHistoryCurrent3 = 50;

float ampHour = 0.0;

float conversionAdcCurrent[4];
float conversionAdcVoltage[4];

float historyCurrent0 [maxHistoryWindow];
float historyCurrent1 [maxHistoryWindow];
float historyCurrent2 [maxHistoryWindow];
float historyCurrent3 [maxHistoryWindow];

float sumCoulomb[4];

const int maxHistoryCoulomb0  = 200;
static int windowHistoryCoulomb0 = 50;

const int maxHistoryVoltage0  = 200;
static int windowHistoryVoltage0 = 50;

const int maxHistoryVoltage1  = 200;
static int windowHistoryVoltage1 = 50;

const int maxHistoryVoltage2  = 200;
static int windowHistoryVoltage2 = 50;

const int maxHistoryVoltage3  = 200;
static int windowHistoryVoltage3 = 50;

int elapsedTimeCurrentDisplay[4];
int elapsedTimeVoltageDisplay[4];
int elapsedTimeCoulombDisplay[4];

int displayChannelCurrentFrequencyIndex[4];
int displayChannelVoltageFrequencyIndex[4];
int displayChannelCoulombFrequencyIndex[4];

int broadcastXDRFrequencyIndex[4];

float historyCurrent[4] [maxHistoryCurrent0];
float historyVoltage[4] [maxHistoryVoltage0];
float historyCoulomb[4] [maxHistoryCoulomb0];

int indexHistoryCurrent[4];
int indexHistoryVoltage[4];
int indexHistoryCoulomb[4];

float minHistoryMagnitudeCurrent[4];
float maxHistoryMagnitudeCurrent[4];

float minHistoryMagnitudeVoltage[4];
float maxHistoryMagnitudeVoltage[4];


float minHistoryMagnitudeCoulomb[4];
float maxHistoryMagnitudeCoulomb[4];

float displayChannelCurrentFrequency[4];
float displayChannelVoltageFrequency[4];
float displayChannelCoulombFrequency[4];

/*
   WiFi broadcast settings
*/

const int UDP_PACKET_SIZE = 80;
char udpBuffer[ UDP_PACKET_SIZE ];

const byte UUIDLENGTH = 36;
char uuid[UUIDLENGTH + 1];

const byte NUUIDLENGTH = 4;
char nuuid[NUUIDLENGTH + 1];

//const char uuid[] = "0a65a21e-8c81-4581-8dcc-3818f3c2d53a";
//const char nuuid[] = "0a65";

char *nameThingWiFiAP = "Thing XXXX";
//uuidStr = ESP8266TrueRandom.uuidToString(uuidNumber);
//  const char n[] = 0a65a21e-8c81-4581-8dcc-3818f3c2d53a
//  nuuidStr = uuidStr.substring(0, 4);

//  strcpy(nameThingWiFiAP, "Thing ");
//  strcat(nameThingWiFiAP, nuuidStr.c_str());


double microsStartTime = 0;

WiFiUDP udp;
WiFiClient espClient;

LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 16 column and 2 r
BigFont02_I2C     big(&lcd); // construct large font object, passing to it the name of our lcd object
#ifdef LCD


//char const *prefixNMEA = "KM"; // Feel free to change this if not suiting your needs

/* Setup WiFi */

#endif // LCD

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire

float kalmanBaselineAccX = 0.0;
float kalmanBaselineAccY = 0.0;
float kalmanBaselineAccZ =   0.0;

int frequencyIndex = 0;


#include <SimpleKalmanFilter.h>

SimpleKalmanFilter kalmanAccX(1, 1, 0.01);
SimpleKalmanFilter kalmanAccY(1, 1, 0.01);
SimpleKalmanFilter kalmanAccZ(1, 1, 0.01);

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveUuid() {
  writeString(50, uuidStr);
}

String loadUuid() {

  String data;
  data = read_String(50, 36);

  data[36] = '\0';

  uuidStr = data;
  nuuidStr = uuidStr.substring(0, 4);
  strcpy(nameThingWiFiAP, "Thing ");
  strcat(nameThingWiFiAP, nuuidStr.c_str());
  //nameThingWiFiAP[36] = '\0';

  return data;


}

String read_String(char add, int maxLength)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < maxLength) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  return String(data);
}

void writeString(char add, String data)
{
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}

void resetUuid() {
  ESP8266TrueRandom.uuid(uuidNumber);
  uuidStr = ESP8266TrueRandom.uuidToString(uuidNumber);
  nuuidStr = uuidStr.substring(0, 4);

  strcpy(nameThingWiFiAP, "Thing ");
  strcat(nameThingWiFiAP, nuuidStr.c_str());

  saveUuid();
}

void tempThingId() {
  ESP8266TrueRandom.uuid(uuidNumber);
  uuidStr = ESP8266TrueRandom.uuidToString(uuidNumber);
  nuuidStr = uuidStr.substring(0, 4);

  strcpy(nameThingWiFiAP, "Thing ");
  strcat(nameThingWiFiAP, nuuidStr.c_str());

}

// https://forum.arduino.cc/t/lcd-bargraph-help-solved/350762
// https://forum.arduino.cc/u/robtillaart

//  METER BARS PRINTING FUNCTION
// passing percentage, x and y position, positions bar length, linear/audio logaritmic bar
// barnum is used to manage times of decayn of each peak separately (see definitions)
void printbar (byte bn, double perc, int xpos, int ypos, int blen, boolean logar)
{
  if ((logar == true) && (perc > 0))   // logaritmic bar
  {
    perc = ( log10(perc ) ) * 50;      // 10 * log10 (value) linear to logaritmic for AUDIO conversion
    if ( perc < 0 ) perc = 0;          // avoid negative values
  }

  a = blen / 99.5 * perc;              // calculate length of bar
  b = 0;

  if ( pkcyc[bn][1] > 0 )              // if PEAK is activated
  {
    if ( (a > (pkval[bn] - 0.01)) || (pkcyc[bn][2] > pkcyc[bn][1]) ) // new peak (w little histeresys) or expiration of peak
    {
      pkval[bn] = a;
      pkcyc[bn][2] = 0;                // reset cycles
    }
    pkcyc[bn][2]++;
  }

  // drawing filled rectangles
  if (a >= 1)
  {
    for (int i = 1; i < a; i++)
    {
      lcd.setCursor(xpos - 1 + i, ypos);
      lcd.write(255);
      b = i;
    }
    a = a - b;
  }
  segment = a * 5;

  // drawing final part of the bar
  if (b < blen)
  {
    lcd.setCursor(xpos + b, ypos);

    switch (segment) {
      case 0:
        lcd.print(" ");
        break;
      case 1:
        lcd.write((byte)0);
        break;
      case 2:
        lcd.write(1);
        break;
      case 3:
        lcd.write(2);
        break;
      case 4:
        lcd.write(3);
        break;
    }
  }

  // cleaning rest of line
  for (int i = 0; i < (blen - b - 1); i++)
  {
    lcd.setCursor(xpos + b + 1 + i, ypos);
    lcd.print(" ");
  }

  b2 = (int) pkval[bn];
  a2 = pkval[bn] - b2;
  segment2 = a2 * 5;

  // DRAWING PEAK
  if ( (pkcyc[bn][1] > 0) && (
         ((b + segment) == 0)                               // if bar empty
         || (b2 > b)                                        // or different box position
         || ( (b2 == b) && segment == 0 && segment2 > 0 )   // special case, too long to explain :-)
       ))
  {
    lcd.setCursor(xpos + b2, ypos);

    switch (segment2) {
      case 0:
        if ( (b2 > 0) || (b2 > b + 1))
        {
          lcd.setCursor(xpos + b2 - 1, ypos);
          lcd.write(7);
        };
        break;
      case 1:
        lcd.write(byte(0));
        break;
      case 2:
        lcd.write(4);
        break;
      case 3:
        lcd.write(5);
        break;
      case 4:
        lcd.write(6);
        break;
    }
  }
}

//  METER BARS PRINTING FUNCTION
// passing percentage, x and y position, positions bar length, linear/audio logaritmic bar
// barnum is used to manage STANDARD/PEAK bar and decayn of each peak separately (see definitions)

void printVerticalBar (byte bn, double perc, int xpos, int ypos, int blen, boolean logar)
{
  if ((logar == true) && (perc > 0))   // logaritmic bar
  {
    perc = ( log10(perc ) ) * 50;  // 10 * log10 (value) linear to logaritmic for AUDIO conversion
    if ( perc < 0 ) {
      perc = 0;  // avoid negative values
    }
  } else {

    if (perc > 99.5) {
      perc = 99.5;
    }

  }

  double a = blen / 99.5 * perc; // calculate length of bar
  b = 0;

  if ( pkcyc[bn][1] > 0 )              // if PEAK is activated
  {
    if ( a > (pkval[bn] - 0.1) | pkcyc[bn][2] > pkcyc[bn][1] ) // new peak (w little histeresys) or expiration of peak
    {
      pkval[bn] = a;
      pkcyc[bn][2] = 0;    // reset cycles
    }

    pkcyc[bn][2]++;
  }

  // drawing main bar
  if (pkcyc[bn][1] == 0)
  {
    drawrest (xpos, ypos, b, a, blen);
  }
  else

    // drawing peak bar
  {
    b2 = (int) pkval[bn];
    a = pkval[bn];
    drawrest (xpos, ypos, b2, a, blen);
  }
}

// trace bar and cleanes (also for peak)
void drawrest (int xpos, int ypos, unsigned char b, double a, int blen)
{
  // drawing filled rectangles
  if (a >= 1) {

    for (int i = 1; i < a; i++) {
      lcd.setCursor(xpos, ypos - i + 1);
      lcd.write(255);
      b = i;
    }
    a = a - b;
  }

  segment = a * 8;
  if (b < blen)
  {
    lcd.setCursor(xpos, ypos - b);

    switch (segment) {
      case 0:
        lcd.print(" ");
        break;
      case 1:
        lcd.write((byte)0);
        break;
      case 2:
        lcd.write(1);
        break;
      case 3:
        lcd.write(2);
        break;
      case 4:
        lcd.write(3);
        break;
      case 5:
        lcd.write(4);
        break;
      case 6:
        lcd.write(5);
        break;
      case 7:
        lcd.write(6);
        break;
    }
  }

  // cleaning rest of line
  for (int i = 0; i < (blen - b - 1); i++)
  {
    lcd.setCursor(xpos, ypos - b - i - 1);
    lcd.print(" ");
  }
}
// Development around the WiFiManager service. Seems to be a reliable place to serve.
void webThing() {

  WiFiClient client = server.available();   // Listen for incoming clients
  boolean jsonFlag = false;


  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");

            if (header.endsWith(".json")) {
              client.println("Content-type:text/json");
              jsonFlag = true;
            } else {
              client.println("Content-type:text/html");
            }
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("GPIO 5 on");
              // output5State = "on";
              // digitalWrite(output5, HIGH);
            } else if (header.indexOf("GET /snapshot.json") >= 0) {
              Serial.println("snapshot");
              jsonFlag = true;
              client.println("{}");

              //    snapshotJson();
              // Clear the header variable
              header = "";
              // Close the connection
              client.stop();
              Serial.println("Client disconnected.");
              Serial.println("");
              return;

              // output5State = "off";
              //  digitalWrite(output5, LOW);
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("GPIO 5 off");
              // output5State = "off";
              //  digitalWrite(output5, LOW);
            } else if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");
              // output4State = "on";
              //  digitalWrite(output4, HIGH);
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");
              //  output4State = "off";
              // digitalWrite(output4, LOW);
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP8266 Web Server</h1>");
            client.println("<p>" + uuidStr + "</p>");
            // Display current state, and ON/OFF buttons for GPIO 5
            //client.println("<p>GPIO 5 - State " + output5State + "</p>");
            // If the output5State is off, it displays the ON button
            // if (output5State=="off") {
            //  client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
            // } else {
            // client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
            //    }

            // Display current state, and ON/OFF buttons for GPIO 4
            //client.println("<p>GPIO 4 - State " + output4State + "</p>");
            // If the output4State is off, it displays the ON button
            //      if (output4State=="off") {
            //      client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
            //  } else {
            //     client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
            //   }
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  //}
}

//void snapshotJson() {

//client.println("{}");

//}

void displayAbout() {

  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("NMEA POWER THING");        // print message at (0, 0)

  lcd.setCursor(0, 1);         // move cursor to   (2, 1)
  lcd.print("stackr.ca");

  lcd.setCursor(0, 2);         // move cursor to   (2, 1)
  lcd.print("17 November 2022");

  lcd.setCursor(0, 3);         // move cursor to   (2, 1)
  lcd.print(nuuidStr);

}

// not used
void configModeCallback (WiFiManager * myWiFiManager) {
  // Serial.println("Entered config mode");
  // Serial.println(WiFi.softAPIP());
  // Serial.println(myWiFiManager->getConfigPortalSSID());
  return;
}

// dev
void setupSPIFFS() {
  /*
    //clean FS, for testing
    //SPIFFS.format();

    //read configuration from FS json
    Serial.println("mounting FS...");

    if (SPIFFS.begin()) {
      Serial.println("mounted file system");
      if (SPIFFS.exists("/config.json")) {
        //file exists, reading and loading
        Serial.println("reading config file");
        File configFile = SPIFFS.open("/config.json", "r");
        if (configFile) {
          Serial.println("opened config file");
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);

          configFile.readBytes(buf.get(), size);
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf.get());
          json.printTo(Serial);
          if (json.success()) {
            Serial.println("\nparsed json");
            strcpy(output, json["output"]);
          } else {
            Serial.println("failed to load json config");
          }
        }
      }
    } else {
      Serial.println("failed to mount FS");
    }
    //end read
  */
}

void webConfigPortal() {
  int timeout = 180;
  WiFiManager wm;

  //reset settings - for testing
  wm.resetSettings();
  // !!!!
  // set configportal timeout
  wm.setConfigPortalTimeout(timeout);

  if (!wm.startConfigPortal(nameThingWiFiAP)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

}

void sendUDP(char *inData)
{
  //  Serial.println("sendUDP");
  //  Serial.println(inData);
  if (broadcastNMEAFlag == false) {
    Serial.println("broadcastNMEAFLag is false");
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {

    udp.beginPacket(ipBroadCast, udpRemotePort);
    udp.write(inData);
    int response = udp.endPacket();

    if (response == 0) {
      Serial.print("sendUDP Did not send packet.\n");
    } else {

    }

  } else {
    Serial.print("No WiFi.\n");
  }
  Serial.print(inData);
  Serial.print("\n");

}

char* substr(char* arr, int begin, int len)
{
  char* res = new char[len + 1];
  for (int i = 0; i < len; i++)
    res[i] = *(arr + begin + i);
  res[len] = 0;
  return res;
}

int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;
  // ignore the first $ sign, no checksum in sentence
  for (i = 1;  i < strlen(nmea_data); i ++) { // remove the - three because no chksum is presnet
    crc ^= nmea_data[i];
  }
  return crc;
}

void defaultSettings() {


  for (int i = 0;  i < 4; i ++) {
    startTimeXDR[i] = 0;

    gainVoltage[i] = 5.01 / 0.41;
    biasVoltage[i] = 0.0;

    conversionAdcCurrent[i] = 0.0078125 / 1000;
    conversionAdcVoltage[i] = 0.125 / 1000;

    gainCurrent[i] = 1;
    biasCurrent[i] = 0.0;

    indexHistoryVoltage[i] = 0;
    indexHistoryCurrent[i] = 0;
    indexHistoryCoulomb[i] = 0;

    minHistoryMagnitudeCurrent[i] = 1e9;
    maxHistoryMagnitudeCurrent[i] = -1e9;

    minHistoryMagnitudeVoltage[i] = 1e9;
    maxHistoryMagnitudeVoltage[i] = -1e9;

    minHistoryMagnitudeCoulomb[i] = 1e9;
    maxHistoryMagnitudeCoulomb[i] = -1e9;

    maxHistoryCurrent[i] = 200;
    windowHistoryCurrent[i] = 50;

    maxHistoryVoltage[i] = 200;
    windowHistoryVoltage[i] = 50;

    maxHistoryCoulomb[i] = 200;
    windowHistoryCoulomb[i] = 50;

    shuntMaxCurrent[i] = 500;
    shuntMaxVoltage[i] = 0.075;

    sumCoulomb[i] = 0.0;

    displayChannelCurrentFrequencyIndex[i] = 2;
    displayChannelVoltageFrequencyIndex[i] = 2;
    displayChannelCoulombFrequencyIndex[i] = 2;

    displayChannelCurrentFrequency[i] = indexToFrequency(displayChannelCurrentFrequencyIndex[i]);
    displayChannelVoltageFrequency[i] = indexToFrequency(displayChannelVoltageFrequencyIndex[i]);
    displayChannelCoulombFrequency[i] = indexToFrequency(displayChannelCoulombFrequencyIndex[i]);

    broadcastXDRFrequencyIndex[i] = 7;
    broadcastXDRFrequency[i] = indexToFrequency(broadcastXDRFrequencyIndex[i]);

    startTimeCurrentDisplay[i] = micros();
    elapsedTimeCurrentDisplay[i] = 0;

    startTimeVoltageDisplay[i] = micros();
    elapsedTimeVoltageDisplay[i] = 0;

    startTimeCoulombDisplay[i] = micros();
    elapsedTimeCoulombDisplay[i] = 0;

  }

}

void sendPMTK(char * data) {
  /*
     Packet Length:
    The maximum length of each packet is restricted to 255 bytes.
    Packet Contents:
    Preamble: 1 byte character. ‘$’
    Talker ID: 4 bytes character string. “PMTK”
    Packet Type: 3 bytes character string. From “000” to “999”
    Data Field: The Data Field has variable length depending on the packet type.
    A comma symbol ‘,’ must be inserted ahead each data field to help the decoder process the
    Data Field.
    : 1 byte character. ‘*’
    The start symbol is used to mark the end of Data Field.
    CHK1, CHK2: 2 bytes character string. CHK1 and CHK2 are the checksum of data between Preamble and ‘*’.
    CR, LF: 2 bytes binary data. (0x0D, 0x0A)
    The 2 bytes are used to identify the end of a packet

  */
  char nmeaData[255];
  char jsonData[255];
  char identityData[255];
  char *dollar = "$";
  char *sentenceNMEA = "PMTK";
  strcpy(nmeaData, dollar);
  //strcat(nmeaData, prefixNMEA);
  strcat(nmeaData, sentenceNMEA);

  strcat(nmeaData, ",");
  strcat(nmeaData, "000");
  strcat(nmeaData, ",");
  strcat(nmeaData, data);

  int crc =  nmea0183_checksum(nmeaData);
  char hex[2];
  sprintf(hex, "%02X", crc);

  strcat(nmeaData, "*");

  strcat(nmeaData, hex);

  sendUDP(nmeaData);

}

void sendXDR(char * prefixNMEA,
             char * sensorTypeA, char * sensorNameA, float sensorValueA, char * sensorUnitsA,
             char * sensorTypeB, char * sensorNameB, float sensorValueB, char * sensorUnitsB,
             char * sensorTypeC, char * sensorNameC, float sensorValueC, char * sensorUnitsC,
             char * sensorTypeD, char * sensorNameD, float sensorValueD, char * sensorUnitsD)
{
  char nmeaData[80];
  char *dollar = "$";
  char *sentenceNMEA = "XDR";
  strcpy(nmeaData, dollar);
  strcat(nmeaData, prefixNMEA);
  strcat(nmeaData, sentenceNMEA);

  strcat(nmeaData, ",");
  strcat(nmeaData, sensorTypeA);
  strcat(nmeaData, ",");
  char append[5];
  sprintf(append, "%.2f", sensorValueA);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, sensorUnitsA);
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameA);
  strcat(nmeaData, ",");

  strcat(nmeaData, sensorTypeB);
  strcat(nmeaData, ",");

  sprintf(append, "%.2f", sensorValueB);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, sensorUnitsB);
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameB);
  strcat(nmeaData, ",");

  strcat(nmeaData, sensorTypeC);
  strcat(nmeaData, ",");
  //  char append[5];
  sprintf(append, "%.2f", sensorValueC);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, sensorUnitsC);
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameC);
  strcat(nmeaData, ",");

  strcat(nmeaData, sensorTypeD);
  strcat(nmeaData, ",");
  sprintf(append, "%.2f", sensorValueD);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, sensorUnitsD);
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameD);

  int crc =  nmea0183_checksum(nmeaData);
  char hex[2];
  sprintf(hex, "%02X", crc);

  strcat(nmeaData, "*");

  strcat(nmeaData, hex);

  sendUDP(nmeaData);



}

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0



void displayVariable(float variable, int column, int row, int width, int precision) {
  char variableString[5];

  dtostrf(variable, width, precision, variableString);

  lcd.setCursor(column, row);
  lcd.print(variableString);
}



void displayQuantityUnit(float quantity, String units, int precision, int column, int row) {
  char variableString[5];

  String multiplierText = "k";
  float multiplier = 1;
  int width = 4 + precision;
  //int precision = 1;
  //Serial.println(quantity);


  if (abs(quantity) < 1e-6) {

    multiplier = 1e-9;
    multiplierText = " p";

  } else   if (abs(quantity) < 1e-3) {

    multiplier = 1e-6;
    multiplierText = " u";

  } else if (abs(quantity) < 0) {

    multiplier = 1e-3;
    multiplierText = " m";

  } else if (abs(quantity) < 1e3) {

    multiplier = 1;
    multiplierText = " ";

  } else if (quantity < 1e6) {

    multiplier = 1e3;
    multiplierText = " k";

  } else if (quantity < 1e9) {

    multiplier = 1e6;
    multiplierText = " M";

  } else if (abs(quantity) < 1e12) {

    multiplier = 1e9;
    multiplierText = " G";

  }

  float x = quantity / multiplier;

  dtostrf(x, width, precision, variableString);

  lcd.setCursor(column, row);
  lcd.print(variableString + multiplierText + units + (multiplier == 1 ? " " : ""));
}

// function to round the number
int roundN(int n, int precision)
{
  // Smaller multiple
  int a = (n / precision) * precision;

  // Larger multiple
  int b = a + precision;

  // Return of closest of two
  return (n - a > b - n) ? b : a;
}


// this function will be called when the button was pressed.
void Click()
{

  if (selectFlag == false && editFlag == false) {
    editFlag = true;
    return;
  }
  if (selectFlag == false && editFlag == true) {
    selectFlag = true;
    return;
  }
  if (selectFlag == true && editFlag == true) {
    selectFlag = false;
    return;
  }
  if (selectFlag == false && editFlag == true) {
    editFlag = false;
    return;
  }

} // Click


// this function will be called when the button was pressed 2 times in a short timeframe.
void doubleClick()
{

} // doubleClick

// this function will be called when the button was pressed 2 times in a short timeframe.
void longPress()
{
  //Serial.println("LONG");

  selectFlag = false;
  editFlag = false;

  saveSettings();

} // doubleClick

void saveSettings() {

  int eeAddress = 0;

  eeAddress += 100;
  EEPROM.put(eeAddress, (bool)broadcastNMEAFlag);



  for (int i = 0; i < 4; i++) {
    eeAddress += 4;
    EEPROM.put(eeAddress, (int)broadcastXDRFrequencyIndex[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, (int)displayChannelVoltageFrequencyIndex[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, (int)displayChannelCurrentFrequencyIndex[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, (int)displayChannelCoulombFrequencyIndex[i]);

  }

  for (int i = 0; i < 4; i++) {
    eeAddress += 4;
    EEPROM.put(eeAddress, biasCurrent[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, gainCurrent[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress,   shuntMaxCurrent[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress,   shuntMaxVoltage[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, biasVoltage[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, gainVoltage[i]);

  }


  boolean ok1 = EEPROM.commit();
  // EEPROM.end();
  // Serial.println((ok1) ? "First commit OK" : "Commit failed");

}

float getFloatEEPROM(int eeAddress, float defaultValue) {
  float floatVariable = 0.0;
  EEPROM.get(eeAddress, floatVariable);
  if (isnan(floatVariable)) {
    floatVariable = defaultValue;
  }
  return floatVariable;

}

int getIntEEPROM(int eeAddress, int defaultValue) {
  int intVariable = 0.0;
  EEPROM.get(eeAddress, intVariable);
  if (isnan(intVariable)) {
    intVariable = defaultValue;
  }
  return intVariable;

}

void loadSettings() {

  int eeAddress = 0;

  eeAddress += 100;
  EEPROM.get(eeAddress, broadcastNMEAFlag);
  if (isnan(broadcastNMEAFlag)) {
    broadcastNMEAFlag = true;
  }

  for (int i = 0; i < 4; i++) {
    eeAddress += 4;
    broadcastXDRFrequencyIndex[i] = getIntEEPROM(eeAddress, broadcastXDRFrequencyIndex[i]);

    eeAddress += 4;
    displayChannelVoltageFrequencyIndex[i] = getIntEEPROM(eeAddress, displayChannelVoltageFrequencyIndex[i]);

    eeAddress += 4;
    displayChannelCurrentFrequencyIndex[i] = getIntEEPROM(eeAddress, displayChannelCurrentFrequencyIndex[i]);

    eeAddress += 4;
    displayChannelCoulombFrequencyIndex[i] = getIntEEPROM(eeAddress, displayChannelCoulombFrequencyIndex[i]);


    broadcastXDRFrequency[i] = indexToFrequency(broadcastXDRFrequencyIndex[i]);
    displayChannelVoltageFrequency[i] = indexToFrequency(displayChannelVoltageFrequencyIndex[i]);
    displayChannelCurrentFrequency[i] = indexToFrequency(displayChannelCurrentFrequencyIndex[i]);
    displayChannelCoulombFrequency[i] = indexToFrequency(displayChannelCoulombFrequencyIndex[i]);
  }

  for (int i = 0; i < 4; i++) {

    eeAddress += 4;
    biasCurrent[i] = getFloatEEPROM(eeAddress, biasCurrent[i]);

    eeAddress += 4;
    gainCurrent[i] = getFloatEEPROM(eeAddress, gainCurrent[i]);

    eeAddress += 4;
    shuntMaxCurrent[i] = getFloatEEPROM(eeAddress, shuntMaxCurrent[i]);

    eeAddress += 4;
    shuntMaxVoltage[i] = getFloatEEPROM(eeAddress, shuntMaxVoltage[i]);

    eeAddress += 4;
    biasVoltage[i] = getFloatEEPROM(eeAddress, biasVoltage[i]);

    eeAddress += 4;
    gainVoltage[i] = getFloatEEPROM(eeAddress, gainVoltage[i]);

  }

}

float magnitudeVector(float x, float y, float z) {

  float magnitude = sqrt(x * x + y * y + z * z);
  return magnitude;

}

void updateEditIndex(int maxEditIndex, bool wrapFlag) {

  editIndex = lastEditIndex + editChangeAmount;
  if (editIndex < 0) {
    editIndex = 0;
  }
  if (editIndex > maxEditIndex) {
    editIndex = maxEditIndex;
  }
}

void updateScrollIndex(int maxScrollIndex, bool wrapFlag) {

  scrollIndex = lastScrollIndex + scrollChangeAmount;
  if (scrollIndex < 0) {
    scrollIndex = 0;
  }
  if (scrollIndex > maxScrollIndex) {
    scrollIndex = maxScrollIndex;
  }
}

float indexToFrequency(int index) {
  return availableUpdateFrequencies[index];
}

void displayVoltageChart(int chan) {

  for ( int i = 0 ; i < 8 ; i++ )
    lcd.createChar( i, blockVertical[i] );

  logar = false;

  maxHistoryMagnitudeVoltage[chan] = 0.0;
  minHistoryMagnitudeVoltage[chan] = 1000.0;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryVoltage[chan] - (i - 1)) + indexHistoryVoltage[chan]) % windowHistoryVoltage[chan];
    float m = historyVoltage[chan][j];

    if (m > maxHistoryMagnitudeVoltage[chan]) {
      maxHistoryMagnitudeVoltage[chan] = m;
    }
    if (m < minHistoryMagnitudeVoltage[chan]) {
      minHistoryMagnitudeVoltage[chan] = m;
    }
  }

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryVoltage[chan] - (i - 1)) + windowHistoryVoltage[chan]) % windowHistoryVoltage[chan];

    float m = historyVoltage[chan][j];

    perc = (float)(m / 0.075 * 100);

    pkcyc[i][1] =  0;
    printVerticalBar (i, perc, 20 - i, 3, 3, false);
  }

  lcd.setCursor(0, 0);
  lcd.print("VOLTAGE ");

  lcd.setCursor(8, 0);
  lcd.print(chan + 1);

  displayEditBrackets(editIndex == 0 && editFlag, 0, 13, 19);
  lcd.setCursor(14, 0);

  if (editFlag) {
    lcd.print(availableUpdateFrequenciesLabels[displayChannelVoltageFrequencyIndex[chan]]);
  } else {
    //displayVariable(historyVoltage[chan][indexHistoryVoltage[chan]], 12, 0, 6, 3);
    displayQuantityUnit(voltage[chan], "V", 1, 12, 0);
    //lcd.setCursor(18, 0);
    //lcd.print(" V");
  }

}

void displayCurrentChart(int chan) {

  for ( int i = 0 ; i < 8 ; i++ )
    lcd.createChar( i, blockVertical[i] );

  logar = false;

  maxHistoryMagnitudeCurrent[chan] = 0.0;
  minHistoryMagnitudeCurrent[chan] = 1000.0;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCurrent[chan] - (i - 1)) + windowHistoryCurrent[chan]) % windowHistoryCurrent[chan];
    float m = historyCurrent[chan][j];

    if (m > maxHistoryMagnitudeCurrent[chan]) {
      maxHistoryMagnitudeCurrent[chan] = m;
    }
    if (m < minHistoryMagnitudeCurrent[chan]) {
      minHistoryMagnitudeCurrent[chan] = m;
    }
  }

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCurrent[chan] - (i - 1)) + windowHistoryCurrent[chan]) % windowHistoryCurrent[chan];

    float m = historyCurrent[chan][j];

    perc = (float)(m / shuntMaxCurrent[chan] * 100);

    pkcyc[i][1] =  0;
    printVerticalBar (i, perc, 20 - i, 3, 3, false);
  }

  lcd.setCursor(0, 0);
  lcd.print("CURRENT ");

  lcd.setCursor(8, 0);
  lcd.print(chan + 1);

  displayEditBrackets(editIndex == 0 && editFlag, 0, 13, 19);
  lcd.setCursor(14, 0);

  if (editFlag) {
    lcd.print(availableUpdateFrequenciesLabels[displayChannelCurrentFrequencyIndex[chan]]);
  } else {
    displayQuantityUnit(current[chan], "A", 1, 12, 0);
  }

}

void displayCoulombChart(int chan) {


  for ( int i = 0 ; i < 8 ; i++ )
    lcd.createChar( i, blockVertical[i] );

  logar = false;

  maxHistoryMagnitudeCoulomb[chan] = -1e9;
  minHistoryMagnitudeCoulomb[chan] = 1e9;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCoulomb[chan] - (i - 1)) + windowHistoryCoulomb[chan]) % windowHistoryCoulomb[chan];
    float m = historyCoulomb[chan][j];

    if (m > maxHistoryMagnitudeCoulomb[chan]) {
      maxHistoryMagnitudeCoulomb[chan] = m;
    }
    if (m < minHistoryMagnitudeCoulomb[chan]) {
      minHistoryMagnitudeCoulomb[chan] = m;
    }
  }

  float chartMaxHistoryMagnitudeCoulomb = maxHistoryMagnitudeCoulomb[chan] * 1.1;
  float chartMinHistoryMagnitudeCoulomb = minHistoryMagnitudeCoulomb[chan] * 0.9;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCoulomb[chan] - (i - 1)) + windowHistoryCoulomb[chan]) % windowHistoryCoulomb[chan];

    float m = historyCoulomb[chan][j];

    perc = (float)((m - minHistoryMagnitudeCoulomb[chan]) / (chartMaxHistoryMagnitudeCoulomb - chartMinHistoryMagnitudeCoulomb) * 100) + 10;

    pkcyc[i][1] =  0;
    printVerticalBar (i, perc, 20 - i, 3, 3, false);
  }

  lcd.setCursor(0, 0);
  lcd.print("COULOMB");

  lcd.setCursor(8, 0);
  lcd.print(chan + 1);

  displayEditBrackets(editIndex == 0 && editFlag, 0, 13, 19);
  lcd.setCursor(14, 0);

  if (editFlag) {
    lcd.print(availableUpdateFrequenciesLabels[displayChannelCoulombFrequencyIndex[chan]]);
  } else {
    displayQuantityUnit(sumCoulomb[chan], "C", 1, 12, 0);

  }

}

void displayEditBrackets(bool editFlag, int row, int startColumn, int endColumn) {
  displayBrackets(editFlag, row, startColumn, endColumn, "[", "]");
  return;
  if (editFlag) {
    lcd.setCursor(startColumn, row);
    lcd.print("[");
    lcd.setCursor(endColumn, row);
    lcd.print("]");

  } else {
    lcd.setCursor(startColumn, row);
    lcd.print(" ");
    lcd.setCursor(endColumn, row);
    lcd.print(" ");
  }
}

void displayActionBrackets(bool editFlag, int row, int startColumn, int endColumn) {
  displayBrackets(editFlag, row, startColumn, endColumn, "<", ">");

}

void displayBrackets(bool editFlag, int row, int startColumn, int endColumn, char* startBracket, char* endBracket) {
  if (editFlag) {
    lcd.setCursor(startColumn, row);
    //lcd.print("[");
    lcd.print(startBracket);
    lcd.setCursor(endColumn, row);
    //lcd.print("]");
    lcd.print(endBracket);

  } else {
    lcd.setCursor(startColumn, row);
    lcd.print(" ");
    lcd.setCursor(endColumn, row);
    lcd.print(" ");
  }
}


bool isUuid(String uuid) {
  // d259e979-e855-430e-8129-6425776b21de
  //String data;
  //9, 14, 19, 24,


  //data = read_String(0);

  char *uuidChar = "";

  strcpy(uuidChar, uuid.c_str());

  bool uuidFlag = true;

  for (int i = 0;  i < 29; i ++) {
    //Serial.println("char" + uuidChar[i]);

    // Dashes in the right places?
    if (i == 8 || i == 13 || i == 18 || i == 23) {

      if (uuidChar[i] == '-') {
        continue;
      } else {
        uuidFlag = false;
        break;
      }

      // Other wise is it a letter or number
      if (isAlphaNumeric(uuidChar[i])) {
        continue;
      }

      uuidFlag = false;
      break;

    }
  }

  return uuidFlag;

}

char mqtt_server[40];

WiFiManagerParameter custom_text("<p>NMEA Air Thing</p>");

//WiFiManagerParameter custom_value("elapsedTimeSensorRead");

WiFiManagerParameter custom_mqtt_server("server", "mqtt_server", mqtt_server, 40);




WiFiManager wifiManager;

void setup()
{

  //setupSPIFFS();
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight

  big.begin();

  Serial.begin(9600);
  while (!Serial) {};
  EEPROM.begin(600);
  loadUuid();

  bool isuuid = isUuid(uuidStr);
  if (!isuuid) {
    Serial.println("No UUID found on Thing.");
    // resetUuid();
    tempThingId();
  } else {
    Serial.println(uuidStr);
  }



  displayAbout();

  if (displayFlag) {
    delay(4000);
  }
  Serial.println("Loading settings from Thing.");
  defaultSettings();
  loadSettings();

  lcd.clear();

  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  Serial.println("Wire interface started.");


  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  button.attachDoubleClick(doubleClick);
  button.attachLongPressStop(longPress);
  button.attachClick(Click);

  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  lcd.setCursor(0, 0);         // move cursor to   (2,
  lcd.print("ADS   Connecting ");

  if (!adsCurrent.begin(0x48)) {
    Serial.println("Could not connect to ADS current device.");
    //while (1);
    adsFlag = false;
  }
  if (adsFlag) {
    adsCurrent.setGain(GAIN_SIXTEEN);

    if (!adsVoltage.begin(0x49)) {
      Serial.println("Failed to initialize ADS Voltage.");
      while (1);
    }
    adsVoltage.setGain(GAIN_ONE);

    lcd.setCursor(0, 0);         // move cursor to   (2,
    lcd.print("ADS   Connected  ");
  }


  //  Serial.println(F("BME680 async test"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a BME680 sensor, check wiring! Proceeding without it."));
    ///while (1);
  }

  // Start SHT3X here

  if (sht.init()) {
    Serial.print("SHT3X  success\n");
  } else {
    Serial.print("SHT3X could not initialise.\n");
    shtFlag = false;
  }
  //sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  sensors.begin();
  /*
    if (sensors.begin()) {

      Serial.print("Dallas OneWire success\n");

    } else {

      Serial.print("Dallas OneWire could not initialise.\n");
      oneWireFlag = false;

    }
  */


  //lcd.clear();
  lcd.setCursor(0, 1);         // move cursor to   (2,
  lcd.print("WIFI  Connecting ");

  nuuidStr = uuidStr.substring(0, 4);

  Serial.print("Got thing's existing access point name");
  Serial.print(nameThingWiFiAP);
  Serial.println();

  strcpy(nameThingWiFiAP, "Thing ");
  strcat(nameThingWiFiAP, nuuidStr.c_str());

  Serial.print("Updated thing's access point name");
  Serial.print(nameThingWiFiAP);
  Serial.println();





  // WiFiManager
  //wifiManager.resetSettings();

  //  WiFiManager wifiManager;
  //    wifiManager.setConfigPortalTimeout(15);
  //  wifiManager.setAPCallback(configModeCallback);
  //if ( digitalRead(D7) == LOW ) {

  lcd.setCursor(0, 1);
  lcd.print("WIFI  Portal opened");
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  bool res;
  wifiManager.setConfigPortalBlocking(false);
  //res = wifiManager.startConfigPortal(nameThingWiFiAP);
  //res = wifiManager.autoConnect(nameThingWiFiAP, "password");

  Serial.print("nameThingWiFiAP ");
  Serial.print(nameThingWiFiAP);
  Serial.println("");

  //wiFiManagerParameter custom_mqtt_server("server","mqtt_server",mqtt_server, 40);
  wifiManager.addParameter(&custom_mqtt_server);

  wifiManager.addParameter(&custom_text);

  //  wifiManager.addParameter(&custom_value);

  WiFiManagerParameter custom_value("hey");
  wifiManager.addParameter(&custom_value);

  //  std::vector<const char *> menu = {"wifi","param","info"};
  //  const char * menu[] = {"wifi","setup","sep","info","exit"};
  std::vector<const char *> menu = {"wifi", "param"};
  wifiManager.setMenu(menu);

  //wifiManager.setCustomHeadElement("<style>html{filter: invert(100%); -webkit-filter: invert(100%);}</style>");

  wifiManager.setCustomHeadElement("<style></style>");

  wifiManager.setShowInfoErase(false);

  //wifiManager.setCustomMenuHTML("Hello");
  // setClass
  // setCustomMenuHTML
  // setMenu
  wifiManager.setTitle("NMEA AIR THING");
  //wifiManager.setParamsPage(true);
  // Not to be combined with setMenu.
  // htmlEntities()
  wifiManager.setCountry("CA");

  wifiManager.setDebugOutput(false);

  res = wifiManager.autoConnect(nameThingWiFiAP);
  if (!res) {
    lcd.setCursor(0, 1);         // move cursor to   (2,
    lcd.print("WIFI  Did not connect");
    Serial.println("Configportal running");

    if (wifiManager.getWiFiSSID() == "") {

      Serial.println("Blank SSID seen");
      //      wifiManager.setConfigPortalBlocking(true);
    }

    // Start up AP as available.
    //  wifiManager.setConfigPortalBlocking(true);

    // set configportal timeout
    //  wifiManager.setConfigPortalTimeout(timeout);
    /*
      if (!wifiManager.startConfigPortal(nameThingWiFiAP)) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
      //    ESP.restart();
        delay(5000);
        Serial.println("foo");
      }
        Serial.println("bar");
    */

  } else {
    lcd.setCursor(0, 1);         // move cursor to   (2,
    lcd.print("WIFI  SSID SET   ");

    Serial.print("Connected to WiFi AP ");
    Serial.print(wifiManager.getWiFiSSID());
    Serial.println();

    sendPMTK("WIFI SSID got");
    // Connected to an SSID.
  }
  if (displayFlag) {
    delay(1000);
  }

  //Serial.println("Before server begin.");
  server.begin();
  //delay(2000);
  //Serial.println("Server begin.");
  /*
    int wifiCount = 0;
    int maxWifiCount = 20;

    int wifiStartTime = micros();

    WiFi.begin();

    while (WiFi.status() != WL_CONNECTED && wifiCount < maxWifiCount) {

      delay(500);
      lcd.setCursor(17, 1);        // move cursor to   (2,

      int wifiElapsedTime = int((micros() - wifiStartTime) / 1e6);
      if (wifiElapsedTime < 10) {
        lcd.print(wifiElapsedTime);
        lcd.print("s ");

      } else {

        lcd.print(".  ");
      }

      if (SerialDebug)
      {
        Serial.print(".");
      }
      wifiCount += 1;


    }
    Serial.println("After WiFi attempt");
    lcd.setCursor(0, 1);         // move cursor to   (2, 1)
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WIFI connected.");
      lcd.print(     "WIFI  Connected     ");
    } else {
      lcd.print(     "WIFI  Not connected    ");
      Serial.println("WIFI Not connected.");
    }
  */
  Serial.println("Switching to 38400 baud monitoring reporting.");

  Serial.begin(38400);

  if (displayFlag) {
    delay(1000);
    lcd.clear();
  }
}
//https://dronebotworkshop.com/wifimanager/



void loop()
{

  wifiManager.setCustomHeadElement("<style>MERP</style>");

  // This is updated whenever the save button is pressed.
  //  Serial.println(custom_mqtt_server.getValue());

  broadcastNMEAFlag = true;

  //WiFiManagerParameter custom_value("hey");

  //  wifiManager.addParameter(&custom_value);
  if (millis() > endTime && sensorReadFlag == true) {

    if (bme.endReading()) {

      sendXDR("TH", "P", "PRSA", bme.pressure / 100.0, "B", "T", "TMPA", bme.temperature, "C", "H", "HMDA", bme.humidity, "P", "X", "GASA", bme.gas_resistance / 1000.0, "X" );
      sensorReadFlag = false;
      wifiManager.setConfigPortalBlocking(false);
    }
  }

  if (oneWireFlag) {

    if (millis() > (endOneWireTime +  1 / sensorOneWireReadFrequency * 1e3))  {

      // call sensors.requestTemperatures() to issue a global temperature
      // request to all devices on the bus
      //Serial.print("Requesting temperatures...");
      sensors.requestTemperatures(); // Send the command to get temperatures
      //Serial.println("DONE");
      // After we got the temperatures, we can print them here.
      // We use the function ByIndex, and as an example get the temperature from the first sensor only.
      float tempC = sensors.getTempCByIndex(0);
      float tempD = sensors.getTempCByIndex(1);

      // Check if reading was successful
      if (tempC != DEVICE_DISCONNECTED_C)
      {
        //Serial.print("Temperature for the device 1 (index 0) is: ");
        //Serial.println(tempC);
        //Serial.println(tempD);

        sendXDR("TH", "T", "TMPP", tempC, "C", "T", "TMPS", tempD, "C", "X", "XXXX", -1, "X", "X", "XXXX", -1, "X" );

      }
      else
      {
        Serial.println("Error: Could not read temperature data");
      }

      endOneWireTime = millis();

    }

    //      sendXDR("TH", "T", "TMPP", tempC, "C", "T", "TMPS", tempD, "C", "X", "XXXX", -1, "X", "X", "XXXX", -1, "X" );

    //endOneWireTime = millis();

  }

  if (shtFlag) {
    if (millis() > (endSHTTime +  1 / sensorSHTReadFrequency * 1e3))  {


      if (sht.readSample()) {
        /*
            Serial.print("SHT:\n");
            Serial.print("  RH: ");
            Serial.print(sht.getHumidity(), 2);
            Serial.print("\n");
            Serial.print("  T:  ");
            Serial.print(sht.getTemperature(), 2);
            Serial.print("\n");
        */
      } else {
        Serial.print("SHTX error in readSample()\n");
      }
      sendXDR("TH", "P", "PRSB", -1, "B", "T", "TMPB", sht.getTemperature(), "C", "H", "HMDB", sht.getHumidity(), "P", "X", "GASB", -1, "X" );
      endSHTTime = millis();
    }
  }
  //  Serial.println("loop");
  wifiManager.process();
  button.tick();

  // Web server (http)/
  //  webThing();
  //////////////////////////////////////////////////////////////////////////////






  WiFiClient client = server.available();   // Listen for incoming clients
  boolean jsonFlag = false;


  if (client) {                             // If a new client connects,
    startWebRequestTime = millis();

    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;




        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");

            if (header.indexOf(".json") >= 0) {
              client.println("Content-type:application/json");
              jsonFlag = true;
            } else {
              client.println("Content-type:text/html");
            }
            client.println("Connection: close");
            client.println();



            float tempC = sensors.getTempCByIndex(0);
            float tempD = sensors.getTempCByIndex(1);

            char jsonData[255];

            char *dollar = "{\"tmpa\":\"";
            char *sentenceNMEA = "XDR";
            strcpy(jsonData, dollar);

            char append[6];
            sprintf(append, "%.2f", tempC);
            strcat(jsonData, append);

            strcat(jsonData, "\",\"tmpb\":\"");

            sprintf(append, "%.2f", tempD);
            strcat(jsonData, append);

            strcat(jsonData, "\"}");

            // turns the GPIOs on and off
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("GPIO 5 on");
              // output5State = "on";
              // digitalWrite(output5, HIGH);
            } else if (header.indexOf("GET /snapshot.json") >= 0) {
              Serial.println("snapshot");
              jsonFlag = true;

              //              client.println("{\"test\":\"json\",\"value\":\"hello\"}");
              client.println(jsonData);
              //    snapshotJson();

              //return;
            } else {



              //if (jsonFlag) {

              //           client.println(jsonData);

              //}

              if (!jsonFlag) {




                // Display the HTML web page
                client.println("<!DOCTYPE html><html>");
                client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                client.println("<link rel=\"icon\" href=\"data:,\">");
                // CSS to style the on/off buttons
                // Feel free to change the background-color and font-size attributes to fit your preferences
                client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
                client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
                client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
                client.println(".button2 {background-color: #77878A;}</style></head>");

                // Web Page Heading
                client.println("<body><h1>NMEA Air Thing</h1>");
                client.println("<p>" + uuidStr + "</p>");


      //          client.println(jsonData);

                client.println("<br />");
                client.print(tempC );                // Display current state, and ON/OFF buttons for GPIO 5
                     client.println("C" );
                client.println("<br />");
                client.print(tempD );
                           client.println("C" );
                                client.println("<br />");

                client.println("<br />");


  if (WiFi.status() == WL_CONNECTED) {
    client.println("WiFi connected.");
  } else {
    client.println("No WiFi.");
  }

                  client.println("<br />");


                
                //client.println("<p>GPIO 5 - State " + output5State + "</p>");
                // If the output5State is off, it displays the ON button
                // if (output5State=="off") {
                //  client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
                // } else {
                // client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
                //    }

                // Display current state, and ON/OFF buttons for GPIO 4
                //client.println("<p>GPIO 4 - State " + output4State + "</p>");
                // If the output4State is off, it displays the ON button
                //      if (output4State=="off") {
                //      client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
                //  } else {
                //     client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
                //   }
runTimeWebRequestTime = millis() - startWebRequestTime;
client.print(runTimeWebRequestTime);
client.println("ms");
                
                client.println("</body></html>");
              }
            }


            
            // The HTTP response ends with another blank line
            client.println();

            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }





  ////////////////////////////////////////////////////////////////////////////////










  if (consoleFlag) {
    //Serial.println("Loop.");
    //Serial.println(nuuidStr);
  }

  int tempChan = 0;

  if (adsFlag) {
    int16_t adcCurrent[4];
    int16_t adcVoltage[4];

    for (int i = 0;  i < 4; i ++) {

      voltageElapsedMillis[i] = micros() - voltageMillis[i];
      voltageMillis[i] = micros();
      adcVoltage[i] = adsVoltage.readADC_SingleEnded(i);

      currentElapsedMillis[i] = micros() - currentMillis[i];
      currentMillis[i] = micros();
      adcCurrent[i] = adsCurrent.readADC_SingleEnded(i);

      voltage[i] = adcVoltage[i] * conversionAdcVoltage[i] * gainVoltage[i] + biasVoltage[i];
      current[i] = adcCurrent[i] * conversionAdcCurrent[i] / shuntMaxVoltage[i] * shuntMaxCurrent[i] * gainCurrent[i] + biasCurrent[i];

      sumCoulomb[i] = sumCoulomb[i] + current[i] * currentElapsedMillis[i] / 1e6;

    }
  }

  static int pos = 0;

  int newPos = encoder->getPosition();


  if (pos != newPos) {
    encoderChangeFlag = true;

    encoderChangeAmount = newPos - pos;

    if (! (editFlag || selectFlag)) {
      screenChangeAmount = screenChangeAmount + newPos - pos;
    }

    if (selectFlag) {
      selectChangeAmount = selectChangeAmount + newPos - pos;
    }

    if (editFlag and !selectFlag) {
      editChangeAmount = editChangeAmount + newPos - pos;
    }

    pos = newPos;

  } // if


  if (displayFlag) {


    // Serial print and/or display at fixed rate independent of data rates
    delt_t = micros() - lastCount;

    // update LCD once per 100ms independent of read rate
    if (delt_t > 100 * 1000)
    {


      screenIndex = lastScreenIndex + screenChangeAmount;
      if (screenIndex < 0) {
        screenIndex = 0;
      }
      if (screenIndex > maxScreenIndex) {
        screenIndex = maxScreenIndex;
      }

      if (screenIndex != lastScreenIndex) {
        lcd.clear();
      }

      /*
         Render LCD page
      */
      switch (screenIndex) {
        case 0:
          selectFlag = false;
          editFlag = false;

          lcd.setCursor(12, 3);
          for (int i = 0;  i < 4; i ++) {

            lcd.setCursor(0, i);
            lcd.print("CH");
            lcd.setCursor(2, i);
            lcd.print(i + 1);

            displayQuantityUnit(current[i], "A", 1, 5, i);
            displayQuantityUnit(voltage[i], "V", 1, 12, i);
          }

          break;

        case 1:
        case 2:
        case 3:
        case 4:

          tempChan = screenIndex - 1;
          selectFlag = false;
          editFlag = false;

          lcd.setCursor(0, 0);
          lcd.print("CHANNEL");

          lcd.setCursor(8, 0);
          lcd.print(tempChan + 1);

          lcd.setCursor(0, 1);
          lcd.print("CURRENT");

          lcd.setCursor(0, 2);
          lcd.print("VOLTAGE");

          lcd.setCursor(0, 3);
          lcd.print("AMPHOUR");

          ampHour = sumCoulomb[tempChan] / 3600;

          displayQuantityUnit(current[tempChan], "A", 2, 8, 1);
          displayQuantityUnit(voltage[tempChan], "V", 3, 8, 2);
          displayQuantityUnit(ampHour, "A-hr", 1, 8, 3);

          break;

        case 5:
        case 6:
        case 7:
        case 8:

          tempChan = screenIndex - 5;
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          displayVoltageChart(tempChan);

          if ((selectFlag || editFlag) && selectChangeAmount != 0) {
            //   switch (editIndex) {
            //     case 0:

            frequencyIndex = displayChannelVoltageFrequencyIndex[tempChan] + selectChangeAmount;

            if (frequencyIndex < 0) {
              frequencyIndex = 0;
            }
            if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
              frequencyIndex = countAvailableUpdateFrequencies - 1;
            }
            displayChannelVoltageFrequency[tempChan] = (float)availableUpdateFrequencies[frequencyIndex];
            //   }

            displayChannelVoltageFrequencyIndex[tempChan] = frequencyIndex;
            displayChannelVoltageFrequency[tempChan] = indexToFrequency(frequencyIndex);
          }

          break;
        case 9:
        case 10:
        case 11:
        case 12:



          tempChan = screenIndex - 9;
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          displayCurrentChart(tempChan);

          if ((selectFlag || editFlag) && selectChangeAmount != 0) {
            //  switch (editIndex) {
            //   case 0:
            frequencyIndex = displayChannelCurrentFrequencyIndex[tempChan] + selectChangeAmount;

            if (frequencyIndex < 0) {
              frequencyIndex = 0;
            }
            if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
              frequencyIndex = countAvailableUpdateFrequencies - 1;
            }
            displayChannelCurrentFrequencyIndex[tempChan] = frequencyIndex;
            displayChannelCurrentFrequency[tempChan] = indexToFrequency(frequencyIndex);
          }
          break;

        case 13:
        case 14:
        case 15:
        case 16:
          tempChan = screenIndex - 13;
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          displayCoulombChart(tempChan);

          if ((selectFlag || editFlag) && selectChangeAmount != 0) {
            //   switch (editIndex) {
            //    case 0:
            frequencyIndex = displayChannelCoulombFrequencyIndex[tempChan] + selectChangeAmount;

            if (frequencyIndex < 0) {
              frequencyIndex = 0;
            }
            if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
              frequencyIndex = countAvailableUpdateFrequencies - 1;
            }
            displayChannelCoulombFrequencyIndex[tempChan] = frequencyIndex;
            displayChannelCoulombFrequency[tempChan] = indexToFrequency(frequencyIndex);


          }

          break;
        case 17:

          updateEditIndex(4, false);
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          if ((editIndex - lastEditIndex) < 0) {
            scrollOffset = scrollOffset - 1;
          } else if ((editIndex - lastEditIndex) > 0) {
            scrollOffset = scrollOffset + 1;
          }

          if (scrollOffset > 2) {
            scrollOffset = 2;
          }
          if (scrollOffset < 0) {
            scrollOffset = 0;
          }

          lcd.setCursor(0, 0);
          lcd.print("NMEA SETTINGS");

          for (int i = 0; i < (2 + 1); i++) {

            displayEditBrackets(scrollOffset == i && editFlag , i + 1, 9, 17);


            switch (editIndex - scrollOffset + i) {
              case 0:
                lcd.setCursor(0, i + 1);
                lcd.print("TRANSMIT");
                break;
              case 1:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH1 ");
                break;
              case 2:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH2 ");
                break;

              case 3:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH3 ");
                break;
              case 4:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH4 ");
                break;

            }

            switch (editIndex - scrollOffset + i) {
              case 0:
                lcd.setCursor(10, i + 1);


                if (broadcastNMEAFlag) {
                  lcd.print("   ON  ");
                } else {

                  lcd.print("  OFF  ");
                }

                break;
              case 1:
              case 2:
              case 3:
              case 4:
                lcd.setCursor(10, i + 1);
                lcd.print(availableUpdateFrequenciesLabels[broadcastXDRFrequencyIndex[editIndex - scrollOffset + i - 1]]);
                break;

            }

            if (editFlag && selectFlag) {

              //saveSettings();
              //selectFlag = false;

            }

          }

          if (editFlag && selectFlag && selectChangeAmount != 0) {

            changeAmount = 0;

            switch (editIndex) {

              case 1:
              case 2:
              case 3:
              case 4:

                frequencyIndex = broadcastXDRFrequencyIndex[editIndex - 1] + selectChangeAmount;


                if (frequencyIndex < 0) {
                  frequencyIndex = 0;
                }
                if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
                  frequencyIndex = countAvailableUpdateFrequencies - 1;

                }

                broadcastXDRFrequencyIndex[editIndex - 1] = frequencyIndex;
                broadcastXDRFrequency[editIndex - 1] = indexToFrequency(frequencyIndex);

                break;
            }

            switch (editIndex) {
              case 0:

                broadcastNMEAFlag = !broadcastNMEAFlag;

                break;


            }
          }

          break;


        case 18:
        case 19:
        case 20:
        case 21:
          tempChan = screenIndex - 18;

          updateEditIndex(5, false);
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          if ((editIndex - lastEditIndex) < 0) {
            scrollOffset = scrollOffset - 1;
          } else if ((editIndex - lastEditIndex) > 0) {
            scrollOffset = scrollOffset + 1;
          }

          if (scrollOffset > 2) {
            scrollOffset = 2;
          }
          if (scrollOffset < 0) {
            scrollOffset = 0;
          }

          lcd.setCursor(0, 0);
          lcd.print("CHANNEL");
          lcd.setCursor(8, 0);
          lcd.print(tempChan + 1);

          lcd.setCursor(10, 0);
          lcd.print("SETTINGS");

          for (int i = 0; i < (2 + 1); i++) {

            displayEditBrackets(scrollOffset == i && editFlag , i + 1, 10, 19);

            switch (editIndex - scrollOffset + i) {
              case 0:
                lcd.setCursor(0, i + 1);
                lcd.print("CURR GAIN");
                break;
              case 1:
                lcd.setCursor(0, i + 1);
                lcd.print("CURR O/S ");
                break;
              case 2:
                lcd.setCursor(0, i + 1);
                lcd.print("SHNT AMPS");
                break;
              case 3:
                lcd.setCursor(0, i + 1);
                lcd.print("SHNT mV  ");
                break;
              case 4:
                lcd.setCursor(0, i + 1);
                lcd.print("VOLT GAIN ");
                break;
              case 5:
                lcd.setCursor(0, i + 1);
                lcd.print("VOLT O/S ");
                break;
            }

            //lcd.setCursor(0,0);
            //lcd.print(editIndex - scrollOffset + i);
            switch (editIndex - scrollOffset + i) {
              case 0:
                displayVariable(gainCurrent[tempChan], 12, i + 1, 6, 2);
                break;
              case 1:
                displayVariable(biasCurrent[tempChan], 12, i + 1, 6, 2);
                break;
              case 2:
                displayVariable(shuntMaxCurrent[tempChan], 12, i + 1, 6, 0);
                break;
              case 3:
                displayVariable(shuntMaxVoltage[tempChan], 12, i + 1, 6, 3);
                break;
              case 4:
                displayVariable(gainVoltage[tempChan], 12, i + 1, 6, 2);
                break;
              case 5:
                displayVariable(biasVoltage[tempChan], 12, i + 1, 6, 2);
                break;

            }



            if (editFlag && selectFlag) {

              //saveSettings();
              //selectFlag = false;
            }

          }

          if (editFlag && selectFlag && selectChangeAmount != 0) {

            changeAmount = 0;

            switch (editIndex) {
              case 0:
                gainCurrent[tempChan] += (float)selectChangeAmount / 100;
                break;
              case 1:
                biasCurrent[tempChan] += (float)selectChangeAmount / 100;
                break;
              case 2:
                shuntMaxCurrent[tempChan] += (float)selectChangeAmount * 5;
                break;
              case 3:
                shuntMaxVoltage[tempChan] += (float)selectChangeAmount / 200;
                break;
              case 4:
                gainVoltage[tempChan] +=  (float)selectChangeAmount / 100;
                break;
              case 5:
                biasVoltage[tempChan] +=  (float)selectChangeAmount / 100;
                break;

            }

          }

          break;

        case 22:
          lcd.setCursor(0,  0); lcd.print("UDP SETTINGS");

          lcd.setCursor(0, 1);
          lcd.print("IP");

          lcd.setCursor(0, 2);
          lcd.print("PORT");

          lcd.setCursor( 5, 1);
          lcd.print(WiFi.localIP());

          lcd.setCursor(5, 2);
          lcd.print(udpRemotePort);

          break;

        case 23:
          updateEditIndex(0, false);
          lcd.setCursor(0, 0);
          lcd.print("WIFI SETTINGS");

          lcd.setCursor(0, 1);
          lcd.print("SSID");

          lcd.setCursor(5, 1);
          lcd.print(WiFi.SSID().substring(0, 15));



          lcd.setCursor(1, 2);
          lcd.print("WEB CONFIG");


          //displayEditBrackets(editIndex == 0 && editFlag, 0 + 1, 0, 19);
          displayActionBrackets(editIndex == 0 && editFlag, 1 + 1, 0, 19);
          //displayEditBrackets(editIndex == 2 && editFlag, 2 + 1, 0, 19);

          if (editFlag && selectFlag) {
            switch (editIndex) {

              case 0:
                lcd.setCursor(1, 2);
                lcd.print("Thing " + nuuidStr + " open");
                webConfigPortal();
                lcd.print("WEB CONFIG OK.     ");
                //resetFunc(); //call reset
                selectFlag = false;

                break;

            }
          }




          break;

        case 24:
          updateEditIndex(2, false);
          lcd.setCursor(0, 0);
          lcd.print("CALIBRATION");


          lcd.setCursor(1, 1);
          lcd.print("RESTART       ");

          lcd.setCursor(1, 2);
          lcd.print("SET DEFAULTS  ");

          lcd.setCursor(1, 3);
          lcd.print("RESET THING UUID");

          displayActionBrackets(editIndex == 0 && editFlag, 0 + 1, 0, 19);
          displayActionBrackets(editIndex == 1 && editFlag, 1 + 1, 0, 19);
          displayActionBrackets(editIndex == 2 && editFlag, 2 + 1, 0, 19);

          if (editFlag && selectFlag) {
            switch (editIndex) {

              case 0:
                lcd.setCursor(1, 1);
                lcd.print("Restarting.      ");
                resetFunc(); //call reset
                // editFlag = false;
                // selectFlag = false;

                break;

              case 1:
                lcd.setCursor(1, 2);
                lcd.print("Setting defaults.   ");
                defaultSettings();
                saveSettings();
                //resetFunc(); //call reset
                editFlag = false;
                selectFlag = false;

                break;

              case 2:
                lcd.setCursor(1, 3);
                lcd.print("Resetting UUID.   ");
                //configWebPortal();
                resetUuid();
                lcd.print("UUID reset.     ");
                //saveSettings();
                //resetFunc(); //call reset
                //            editFlag = false;
                selectFlag = false;
                editFlag = false;

                break;

            }
          }

          break;
        case 25:
          displayAbout();
          break;

        default:
          break;
      }


      lastScreenIndex = screenIndex;
      lastEditIndex = editIndex;
      lastSelectIndex = selectIndex;

      lastCount = micros();

      encoderChangeFlag = false;
      encoderChangeAmount = 0;

      if (! (editFlag || selectFlag)) {
        screenChangeAmount = 0;
      }

      if (selectFlag) {
        selectChangeAmount = 0;
      }

      if (editFlag) {
        editChangeAmount = 0;
      }

    } // if (mpu.delt_t > 500)

  }


  elapsedTimeSensorRead = micros() - startTimeSensorRead;
  if (elapsedTimeSensorRead > 1 / sensorReadFrequency * 1e6) {
    //   Serial.println("Started BME reading.");
    endTime = bme.beginReading();
    startTimeSensorRead = micros();
    sensorReadFlag = true;
  }








  //  int elapsedTimeNMEA = micros() - startTimeNMEA;
  if (adsFlag) {
    for (int i = 0;  i < 4; i ++) {
      elapsedTimeCurrentDisplay[i] = micros() - startTimeCurrentDisplay[i];
      elapsedTimeVoltageDisplay[i] = micros() - startTimeVoltageDisplay[i];
      elapsedTimeCoulombDisplay[i] = micros() - startTimeCoulombDisplay[i];
      elapsedTimeXDR[i] = micros() - startTimeXDR[i];
    }


    for (int i = 0;  i < 4; i ++) {

      if (elapsedTimeXDR[i] > 1 / broadcastXDRFrequency[i] * 1e6) {
        startTimeXDR[i] = micros();


        char number[5];
        sprintf(number, "%d", i);

        char ampLabel[5];
        strcpy(ampLabel, "AMP");
        strcat(ampLabel, number);

        char voltLabel[5];
        strcpy(voltLabel, "VLT");
        strcat(voltLabel, number);


        char coulombLabel[5];
        strcpy(coulombLabel, "CLB");
        strcat(coulombLabel, number);

        sendXDR("TH", "X", ampLabel, current[i], "X", "X", voltLabel, voltage[i], "X", "X", coulombLabel, sumCoulomb[i], "X", "X", "BLNK", 0.0, "X");
      }
    }

    for (int i = 0;  i < 4; i ++) {

      if (elapsedTimeCurrentDisplay[i] > 1 / displayChannelCurrentFrequency[i] * 1e6) {
        startTimeCurrentDisplay[i] = micros();

        indexHistoryCurrent[i] = (indexHistoryCurrent[i] + 1) % windowHistoryCurrent[i];
        historyCurrent[i][indexHistoryCurrent[i]] = current[i];
      }


      if (elapsedTimeVoltageDisplay[i] > 1 / displayChannelVoltageFrequency[i] * 1e6) {
        startTimeVoltageDisplay[i] = micros();

        indexHistoryVoltage[i] = (indexHistoryVoltage[i] + 1) % windowHistoryVoltage[i];
        historyVoltage[i][indexHistoryVoltage[i]] = voltage[i];

      }

      if (elapsedTimeCoulombDisplay[i] > 1 / displayChannelCoulombFrequency[i] * 1e6) {

        startTimeCoulombDisplay[i] = micros();

        indexHistoryCoulomb[i] = (indexHistoryCoulomb[i] + 1) % windowHistoryCoulomb[i];
        historyCoulomb[i][indexHistoryCoulomb[i]] = sumCoulomb[i];

      }
    }
  }
}

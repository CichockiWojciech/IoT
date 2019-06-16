#include <Arduino.h>
#include "Thing.h"
#include "WebThingAdapter.h"

// Wire and SPI commiunication
#include <Wire.h>
#include <SPI.h>
// Ada fruiud
#include <Adafruit_Sensor.h>
// Light sensor
#include <Adafruit_TSL2561_U.h>
#include "TSl2581.h"
// BME 280
#include <Adafruit_BME280.h>
// servo
#include "Servo.h"

/** GPIO */
#define D1 (5)
#define D2 (4)
#define D5 (14)
#define D6 (12)
#define EXTERNAL_LED D6
#define SERVO D5

const char* ssid = "J5";
const char* password = "12345678";

WebThingAdapter* adapter;

/**Build-in Led id */
#if defined(LED_BUILTIN)
const int ledPin = LED_BUILTIN;
#else
const int ledPin = 13;  // manually configure LED pin
#endif

const char* ledTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingDevice led("led", "Built-in LED", ledTypes);
ThingProperty ledOn("on", "", BOOLEAN, "OnOffProperty");
bool lastOn = false;

/* External Led id */
//const char* extLedTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingDevice extLed("extLed", "Smart LED", ledTypes);
ThingProperty extLedOn("on", "", BOOLEAN, "OnOffProperty");
ThingProperty extLedAutoProperty("Auto Control", "Auto Control", BOOLEAN, "Light sensor control");
ThingProperty extLedLuxBoundProperty("Lux boundary", "Light ", NUMBER, nullptr);

/** Light Sensor*/
const char* lightSensorTypes[] = {"MultiLevelSensor","Sensor", nullptr};
ThingDevice lightSensor("lux", "Light intensity", lightSensorTypes);
ThingProperty luxProperty("light intensity", "Analog Input pin", NUMBER, "LevelProperty");

WaveShare_TSL2581 tsl = WaveShare_TSL2581();
int TSL2581_INT = 13;
void read_id(void);
void Read_gpio_interrupt(uint16_t mindata, uint16_t maxdata);
const int address = TSL2561_ADDR_FLOAT;

const int defualtServoBound = 500;

/** Servo id */
const char* servoTypes[] = {"servoOnOff", "turn", nullptr};
ThingDevice servoDevice("servo", "Blinds", servoTypes);
ThingProperty servoProperty("height level", "Analog Input pin", NUMBER, "LevelProperty");
ThingProperty servoAutoProperty("Auto Control", "Auto Control", BOOLEAN, "Light sensor control");
ThingProperty servoLuxBoundProperty("Lux boundary", "Light ", NUMBER, nullptr);
Servo servo;

int pos = 0;
int lastPos = 0;
int TIME_180_RIGHT = 415;
int TIME_180_LEFT = 465;

void set(int x);
int sclatePosition(int readPosition){ return readPosition * 720 / 100; }

// BME
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

const char* bmeSensorTypes[] = {"MultiLevelSensor","Sensor", nullptr};
ThingDevice tempDevice("temp", "Temperature", bmeSensorTypes);
ThingDevice pressureDevice("bar", "Pressure", bmeSensorTypes);

ThingProperty tempProperty("Temperature", "Analog Input pin", NUMBER, "LevelProperty");
ThingProperty pressureProperty("Pressure", "Analog Input pin", NUMBER, "LevelProperty");

Adafruit_BME280 bme; // I2C

void initBME();
void printDevicesURL();

/** Things Property Values */
ThingPropertyValue numberProperty;

void setup(void){
  // led
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // external led
  pinMode(EXTERNAL_LED, OUTPUT);
  digitalWrite(EXTERNAL_LED, LOW);

  // light sensor
  Wire.begin(); //i2c config
  pinMode(TSL2581_INT, INPUT);      // sets the digital pin 7 as input
  /* Initialise the sensor */
  read_id();
  /* Setup the sensor power on */
  tsl.TSL2581_power_on();
  delay(500);
  //  /* Setup the sensor gain and integration time */
  tsl.TSL2581_config();
  
  Serial.begin(115200);
  Serial.println("");
  Serial.print("Connecting to \"");
  Serial.print(ssid);
  Serial.println("\"");
#if defined(ESP8266) || defined(ESP32)
  WiFi.mode(WIFI_STA);
#endif
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  bool blink = true;
  delay(500);
  Serial.print(".");
  digitalWrite(ledPin, blink ? LOW : HIGH); // active low led
  blink = !blink;
  
  digitalWrite(ledPin, HIGH); // active low led

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  adapter = new WebThingAdapter("w25", WiFi.localIP());

  //set up external led
  extLed.addProperty(&extLedOn);
  extLed.addProperty(&extLedAutoProperty);
  extLed.addProperty(&extLedLuxBoundProperty);
  adapter->addDevice(&extLed);
  delay(100);

  // set up led
  led.addProperty(&ledOn);
  adapter->addDevice(&led);
  delay(100);

  // set up light sensor
  lightSensor.addProperty(&luxProperty);
  adapter->addDevice(&lightSensor);
  delay(100);

  //set up servo
  servo.attach(SERVO);
  servo.write(90);
  servoDevice.addProperty(&servoProperty);
  servoDevice.addProperty(&servoAutoProperty);
  servoDevice.addProperty(&servoLuxBoundProperty);
  adapter->addDevice(&servoDevice);
  delay(100);
  
  //set up bme
  initBME();

  adapter->begin();
  Serial.println("HTTP server started");

  // inti servo
  numberProperty.number = defualtServoBound;
  servoLuxBoundProperty.setValue(numberProperty);

  // print all URLs
  printDevicesURL();
}

void loop() {  
  adapter->update();

  // handel build-in led
  bool on = ledOn.getValue().boolean;
  digitalWrite(ledPin, on ? LOW : HIGH); // active low led

  // handel light sensor
  unsigned long lux = 0;
  tsl.TSL2581_Read_Channel();
  lux = tsl.calculateLux(2, NOM_INTEG_CYCLE);
  Read_gpio_interrupt(2000, 50000);
  numberProperty.number = lux;
  luxProperty.setValue(numberProperty);

  // handel extern led
  int ledAuto = extLedAutoProperty.getValue().boolean;
  if(ledAuto){
    int luxBound = extLedLuxBoundProperty.getValue().number;
    if(lux > luxBound)
      digitalWrite(EXTERNAL_LED, LOW);
    else
      digitalWrite(EXTERNAL_LED, HIGH);
  }else{
    bool extOn = extLedOn.getValue().boolean;
    digitalWrite(EXTERNAL_LED, extOn ? HIGH : LOW ); // active high led
  }

  // handle servo
  int servoPos = 0;
  boolean servoAuto = servoAutoProperty.getValue().boolean;
  if(servoAuto){
    int luxBound = servoLuxBoundProperty.getValue().number;
    if(luxBound == 0) luxBound = defualtServoBound;
    if(luxBound < 0) luxBound = defualtServoBound;
    servoPos = (double)lux / luxBound * 100.0;
    if(servoPos > 100) servoPos = 100;
    servoPos = 100 - servoPos;
  }else{
    servoPos = servoProperty.getValue().number; 
  }
  servoPos = sclatePosition(servoPos);
  if(lastPos != servoPos){
    lastPos = servoPos;
    set(servoPos);
  }

  handleBME();
}

void read_id(void)
{
  int   id;
  int a;
  id = tsl.TSL2581_Read_ID();
  a = id & 0xf0;      //The lower four bits are the silicon version number
  if (!(a == 144))    //ID = 90H = 144D
  {
    Serial.println("false ");
  } else {
    Serial.print("I2C DEV is working ,id = ");
    Serial.println(id);
    delay(500);
  }
}

void Read_gpio_interrupt(uint16_t mindata, uint16_t maxdata){
  tsl.SET_Interrupt_Threshold(mindata, maxdata);
  int val = digitalRead(TSL2581_INT);
  if (val == 1)
  {
  } else {
    tsl.Reload_register();
  }
}

 void set(int x){
  if(pos == x)
    return;
  int dist = abs(pos - x);
  if(pos > x){
    servo.write(75);
    delay(TIME_180_LEFT * dist / 180);
  }
  else{
    servo.write(105);
    delay(TIME_180_RIGHT * dist / 180);
  }
  servo.write(90);
  pos = x;
 }

void initBME(){  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  //set up bmp
  tempDevice.addProperty(&tempProperty);
  adapter->addDevice(&tempDevice);

  pressureDevice.addProperty(&pressureProperty);
  adapter->addDevice(&pressureDevice);
}

void handleBME() {
  ThingPropertyValue value;
  
  value.number = bme.readTemperature();
  tempProperty.setValue(value);
  
  value.number = bme.readPressure() / 100.0F;
  pressureProperty.setValue(value);
}

void printDevicesURL(){
  // print led URL
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(led.id);

  // print external led URL
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(extLed.id);

  // print external led URL
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(lightSensor.id);

  // print servo URL
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(servoDevice.id);

  // print tempDevice URL
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(tempDevice.id);

  // print pressureDevice URL
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(pressureDevice.id);
}

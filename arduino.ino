#include <ArduinoBLE.h>
#include <Wire.h>
#include <SPI.h>
#include "SparkFunBME280.h"
#include <SparkFun_VEML6075_Arduino_Library.h>
#include "SparkFun_AS3935.h"

BME280 tempSensor;
VEML6075 uv;
SparkFun_AS3935 lightning;

#define INDOOR 0x12 
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01

#if defined(ESP_PLATFORM)
int LED_BUILTIN = 5;
//int A0 = 34;
int G0 = 4;
int D0 = 23;
//int A1 = 35;
int D1 = 27;
const int G3 = 17;
int G1 = 12;
#elif defined(ARDUINO_ARCH_SAMD)
int G0 = 2;
int D0 = 0;
int D1 = 1;
//const int lightningInt = G3;  //SPI does not currently work on the SAMD51 proto
//int spiCS = G1;
#endif

int soilPin = A0;  //Pin number that measures analog moisture signal
int soilPower = G0;  //Pin number that will power the soil moisture sensor
int WSPEED = D0; //Digital I/O pin for wind speed
int WDIR = A1; //Analog pin for wind direction
int RAIN = D1;   //Digital I/O pin for rain fall
const int lightningInt = G3; // Interrupt pin for lightning detection
int spiCS = G1; //SPI chip select pin

volatile bool rainFlag = false;
volatile bool windFlag = false;

//Function is called every time the rain bucket tips
void rainIRQ()
{
  rainFlag = true;
}

//Function is called when the magnet in the anemometer is activated
void wspeedIRQ()
{
  windFlag = true;
}

// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector. 
int intVal = 0;
int noise = 2; // Value between 1-7 
int disturber = 2; // Value between 1-10

// UUIDs for the BLE service and characteristic
#define BLE_UUID_SERVICE "12345678-1234-1234-1234-123456789ABC"
#define BLE_UUID_CHARACTERISTIC "87654321-4321-4321-4321-CBA987654321"

BLEService customService(BLE_UUID_SERVICE); // Create a BLE service
BLEStringCharacteristic customCharacteristic(BLE_UUID_CHARACTERISTIC, BLERead | BLENotify, 20); // Create a BLE characteristic

void setup() {
  Serial.begin(115200); // Start serial communication at 9600 baud rate
  while (!Serial);

  if (!BLE.begin()) { // Initialize BLE
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Arduino BLE"); // Set the local name
  BLE.setAdvertisedService(customService); // Advertise the custom service
  customService.addCharacteristic(customCharacteristic); // Add the characteristic
  BLE.addService(customService); // Add the service

  customCharacteristic.writeValue("Hello, BLE!"); // Set the initial value

  BLE.advertise(); // Start advertising

  Serial.println("Bluetooth device is now advertising");

  Serial.println("MicroMod Weather Carrier Board Test");
  Serial.println();

  Wire.begin();
  SPI.begin();

  if (tempSensor.beginI2C() == false) { //Begin communication over I2C
    Serial.println("BME280 did not respond.");
    while(1); //Freeze
  }
  if (uv.begin() == false) {
    Serial.println("VEML6075 did not respond.");
    while(1);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(soilPower, OUTPUT);
  digitalWrite(soilPower, LOW);
  // When lightning is detected the interrupt pin goes HIGH.
  pinMode(lightningInt, INPUT);

  //Initialization for weather meter
  pinMode(WSPEED, INPUT_PULLUP);  //Input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP);    //Input from wind meters rain gauge sensor
  //attach external interrupt pins to IRQ functions
  attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);
  //turn on interrupts
  interrupts();

  if(lightning.beginSPI(spiCS, 2000000) == false){ 
    Serial.println ("Lightning Detector did not start up, freezing!"); 
    while(1); 
  }
  else
    Serial.println("Schmow-ZoW, Lightning Detector Ready!");

  // The lightning detector defaults to an indoor setting at 
  // the cost of less sensitivity, if you plan on using this outdoors 
  // uncomment the following line:
  lightning.setIndoorOutdoor(OUTDOOR); 
}

void loop() {
  BLEDevice central = BLE.central(); // Check for a central device connection

  if (central) { // If a central device has connected
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    // Send data while the central device is connected
    while (central.connected()) {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      customCharacteristic.writeValue("Connected");
      Serial.println();
      Serial.print("Temperature: ");
      Serial.println(tempSensor.readTempF(), 2);
      Serial.print("Humidity: ");
      Serial.println(tempSensor.readFloatHumidity(), 0);
      Serial.print("Pressure: ");
      Serial.println(tempSensor.readFloatPressure(), 0);
      Serial.print("Altitude: ");
      Serial.println(tempSensor.readFloatAltitudeFeet(), 1);

      Serial.print("UV A, B, index: ");
      Serial.println(String(uv.uva()) + ", " + String(uv.uvb()) + ", "+ String(uv.index()));

      Serial.print("Soil Moisture = ");
      Serial.println(readSoil());

      Serial.print("Wind direction: ");
      Serial.print(getWindDirection());
      Serial.println(" degrees");
      //Check interrupt flags
      if (rainFlag == true){
        Serial.println("Rain click!");
        rainFlag = false;
      }
      if (windFlag == true){
        Serial.println("Wind click!");
        windFlag = false;
      }

      // Hardware has alerted us to an event, now we read the interrupt register
      if(digitalRead(lightningInt) == HIGH){
        intVal = lightning.readInterruptReg();
        if(intVal == NOISE_INT){
          Serial.println("Noise."); 
          // Too much noise? Uncomment the code below, a higher number means better
          // noise rejection.
          //lightning.setNoiseLevel(noise); 
        }
        else if(intVal == DISTURBER_INT){
          Serial.println("Disturber."); 
          // Too many disturbers? Uncomment the code below, a higher number means better
          // disturber rejection.
          //lightning.watchdogThreshold(disturber);  
        }
        else if(intVal == LIGHTNING_INT){
          Serial.println("Lightning Strike Detected!"); 
          // Lightning! Now how far away is it? Distance estimation takes into
          // account any previously seen events in the last 15 seconds. 
          byte distance = lightning.distanceToStorm(); 
          Serial.print("Approximately: "); 
          Serial.print(distance); 
          Serial.println("km away!"); 
          customCharacteristic.writeValue("Lightning detected!");
        }
      }

      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

      delay(3000);
    }

    Serial.println("Central device disconnected");
  }
}

int readSoil() {
  int moistVal = 0;  //Variable for storing moisture value
  //Power Senor
  digitalWrite(soilPower, HIGH);
  delay(10);
  moistVal = analogRead(soilPin);  //Read the SIG value from sensor
  digitalWrite(soilPower, LOW); //Turn the sensor off
  return moistVal; //Return current moisture value
}

int getWindDirection()
{
  unsigned int adc;
  adc = analogRead(WDIR); //get the current readings from the sensor

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1);
}
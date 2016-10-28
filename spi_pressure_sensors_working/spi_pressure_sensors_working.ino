#include <SPI.h>

// Arduino Uno
//const int chipSelectPinA = 10; // "cs/ss" pin for selecting which sensor to address.
//const int chipSelectPinB = 9;

// Light Blue Bean
const int chipSelectPinA = 2; // "cs/ss" pin for selecting which sensor to address.
const int chipSelectPinB = 1;

byte pressureDataA;
byte pressureDataB;

//int bothBytes;
int combinedInts;

float psiOutput;

uint32_t sleep_duration_ms = 3500; //3.5 seconds of sleep time

void setup() {
  Serial.begin(9600); // or 9600
  SPI.begin();
  SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0));

  pinMode(chipSelectPinA, OUTPUT);
  pinMode(chipSelectPinB, OUTPUT);

  digitalWrite(chipSelectPinA, HIGH);
  digitalWrite(chipSelectPinB, HIGH);

  delay(500);
  
}

void loop() {

  // Get pressure data
  pressureDataA = readSensor(chipSelectPinA, 2);
  pressureDataB = readSensor(chipSelectPinB, 2);
  Serial.println();
  //delay(3000); // wait three seconds before looping again
  Bean.sleep(sleep_duration_ms);

  //Serial.println(combinedInts);

}

// Function for reading the SPI pressure sensor
unsigned int readSensor(int whichChip, int bytesToRead) {
  byte firstByte;
  byte secondByte;

  // asserting this sensor by bringing CS pin low
  digitalWrite(whichChip, LOW);

  // here we're asking for our two bytes
  firstByte = SPI.transfer(0x00);
  secondByte = SPI.transfer(0x00);

  // here we're bitshifting the first byte 8 bits to the left
  uint16_t bothBytes = (firstByte << 8) | secondByte;
  //Serial.println(bothBytes);
  transferFunction(bothBytes);
  
  // de-asserting this sensor by bringing CS pin high
  digitalWrite(whichChip, HIGH);
  //return (combinedInts);
  return (bothBytes);
  
}

// Function for converting sensor output to PSI readings
float transferFunction(uint16_t dataIn) {
  float outputMax = 16384.0; // 2 to the 14th power (from sensor's data sheet)
  float outputMin = 0.0;
  float pressureMax = 30.0; // max 30 psi (from sensor's datea sheet)
  float pressureMin = 0.0;
  float pressure = 0.0;
  // from data sheet "output" in this case, is passed as dataIn
  
  // transfer function: using output to solve for pressure
  pressure = (dataIn - outputMin) * (pressureMax - pressureMin) / (outputMax - outputMin);
  pressure = pressure + pressureMin;

  //psiOutput = float(pressure);
  psiOutput = pressure;
  Serial.println(psiOutput);
  return (psiOutput);

}











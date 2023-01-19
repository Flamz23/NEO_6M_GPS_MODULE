#include <AltSoftSerial.h>
#include <TinyGPS++.h>

// define requireed pins
const byte LATCH = 3;  // Connected to ST_CP of 74HC595
const byte CLK = 5;  // Connected to SH_CP of 74HC595
const byte DATA = 2;  // Connected to DS of 74HC595

const byte D1 = A1; // digit controller
const byte D2 = A4;
const byte D3 = A2;
const byte D4 = A0;

byte digits[4] = {D1, D2, D3, D4};
byte numCharacters[10] = {B10101111, // byte array for 7 seg digit arrangement
                          B10100000,
                          B11001101,
                          B11101001,
                          B11100010,
                          B01101011,
                          B01101111,
                          B10100001,
                          B11101111,
                          B11100011
                         };
const int displayInterval = 4;
int vehicleSpeed = 0;

AltSoftSerial gpsSerial(13, A3); //rx,tx
TinyGPSPlus gps;


void resetDigits() {
  digitalWrite(D1, HIGH);
  digitalWrite(D2, HIGH);
  digitalWrite(D3, HIGH);
  digitalWrite(D4, HIGH);

  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLK, LSBFIRST, B0000000);
  digitalWrite(LATCH, HIGH);
}

// displays a number at the specified digit
void displayDigit(int _digit, int _number) {
  resetDigits();

  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLK, LSBFIRST, numCharacters[_number]); // shift out the bits MSB first {B11111111, 0xcc, 255}
  digitalWrite(LATCH, HIGH); // set latch high to set output pins

  digitalWrite(digits[_digit], LOW);
}

void writeNum(int _value) {
  //get digits from MSD to LSD from value parameter
  int valueFirstDigit  = _value % 10;
  int valueSecondDigit  = (_value / 10) % 10;
  int valueThirdDigit  = (_value / 100) % 10;
  int valueFourthDigit  = (_value / 1000) % 10;

  displayDigit(0, valueFirstDigit);
  delay(displayInterval); // delay between digits to set duty cycle

  if (_value > 9) {
    displayDigit(1, valueSecondDigit);
    delay(displayInterval);
  }

  // displays leading zeroes
  if (_value > 99) {
    displayDigit(2, valueThirdDigit);
    delay(displayInterval);
  }

  if (_value > 999) {
    displayDigit(3, valueFourthDigit);
    delay(displayInterval);
  }
}

void setup() {
  // Inittialize output pins
  pinMode(LATCH, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DATA, OUTPUT);

  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);

  Serial.begin(9600);
  gpsSerial.begin(9600); // connect gps sensor
  writeNum(0);
}

void loop() {
//  while (gpsSerial.available() > 0)
//  {
//    Serial.write(gpsSerial.read());
//    gps.encode(gpsSerial.read());
//    if (gps.location.isUpdated())
//    {
//      Serial.print("Latitude: ");
//      Serial.print(gps.location.lat(), 6); //Getting Latitude
//      Serial.print(" Longitude: ");
//      Serial.print(gps.location.lng(), 6); //Getting Longitude
//      Serial.print(" speed: ");
//      Serial.println(gps.speed.mph()); //Getting speed
//      Serial.print("altitude: ");
//      Serial.println(gps.altitude.miles()); //Getting speed
//
//      vehicleSpeed = gps.altitude.miles();
//      writeNum(vehicleSpeed);
//
//      // Number of satellites in use
//      Serial.print("Number os satellites in use : ");
//      Serial.println(gps.satellites.value());
//    }
//  }

  while(gpsSerial.available() > 0)     // While there is data on the RX pin...
  {
      char c = gpsSerial.read();    // load the data into a variable...
      Serial.write(c) ;
  }

//  while (gpsSerial.available() <= 0) {
//    writeNum(vehicleSpeed);
//  }
}

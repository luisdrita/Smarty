// --------------------------------------------------------------- Including Libraries -----------------------------------------------------

#include <NeoSWSerial.h> // Extra serial communication port
#include <TinyGPS++.h> // GPS module
#include <Servo.h> // Servo

// GPS Parameters
static const int RXPin = 12;
static const uint32_t GPSBaud = 9600;

// The serial connection to the GPSS device
NeoSWSerial ss (RXPin,13);

// Creating Instances
Servo servo;
TinyGPSPlus gps;

// ------------------------------------------------------------------ Defining Pins --------------------------------------------------------

int ledPin1 = 2;
int ledPin2 = 3;
int headLights = 5;
int backLights = 7;

boolean ledPin1State = LOW;
boolean ledPin2State = LOW;

int buzzer = 4;

int motorVelocityControl = 11;
int motorFrontDirectionControl = 9;
int motorBackDirectionControl = 10;

double aux;

int state;
int state2 = 4;

int x, y = 0;
int xAxis, yAxis;

unsigned long previousMillis = 0;
const long interval = 200;

// --------------------------------------------------------------------- Setup ------------------------------------------------------------

void setup() {

  // --------------------------------------------------------------- Pin Mode (Input/Output)

  pinMode(ledPin1, OUTPUT); // Side Light
  pinMode(ledPin2, OUTPUT); // Side Light
  pinMode(backLights, OUTPUT); // Back Lights
  pinMode(headLights, OUTPUT); // Front Lights

  pinMode(motorVelocityControl, OUTPUT);
  pinMode(motorFrontDirectionControl, OUTPUT);
  pinMode(motorBackDirectionControl, OUTPUT);

  servo.attach (6);
  
  Serial.begin(38400);
  ss.begin(GPSBaud);
  
}

// -------------------------------------------------------------------- Loop ------------------------------------------------------------

void loop() {
  
unsigned long currentMillis = millis();

  // --------------------------------------------------------------- GPS Module

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
    
      if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print("|");
    Serial.println(gps.location.lng(), 6);
  }

  // --------------------------------------------------------------- Bluetooth Module

  while (Serial.available() >= 2) {

    x = Serial.read();

    delay(10);

    y = Serial.read();

  };

  delay(10);

  // --------------------------------------------------------------- Head Lights

  if (x == 1) {

    digitalWrite(headLights, HIGH);

  }

  if (x == 0) {

    digitalWrite(headLights, LOW);

  }

  // --------------------------------------------------------------- Back Lights

  if (motorBackDirectionControl == HIGH) {

    digitalWrite(backLights, HIGH);

  }  else {

    digitalWrite(backLights, LOW);

  }

  // --------------------------------------------------------------- Buzzer
  
  if (x == 2) {

    tone(buzzer, 200, 80);

  } else if (x == 3) {

    digitalWrite(buzzer, LOW);

  }

  // --------------------------------------------------------------- Left Light

  if (x == 5) {

    if (currentMillis - previousMillis >= interval) {

      previousMillis = currentMillis;

      if (ledPin1State == LOW) {
        digitalWrite(ledPin1, HIGH);
        ledPin1State = HIGH;
      } else {
        digitalWrite(ledPin1, LOW);
        ledPin1State = LOW;
      }

    }

  } else if (x == 4) {

    digitalWrite(ledPin1, LOW);

  }

  // --------------------------------------------------------------- Right Light
  
  if (x == 6) {

    if (currentMillis - previousMillis >= interval) {

      previousMillis = currentMillis;

      if (ledPin2State == LOW) {
        digitalWrite(ledPin2, HIGH);
        ledPin2State = HIGH;
      } else {
        digitalWrite(ledPin2, LOW);
        ledPin2State = LOW;
      }

    }

  } else if (x == 7) {

    digitalWrite(ledPin2, LOW);

  }

  // --------------------------------------------------------------- Servo Motor

  if ((x == 255 && y == 255) || x == 153) {

    servo.write(90 - 15);
    delay(10);

  } else if (x > 153 & y < 153) {
    servo.write((180 / PI)*asin((153 - y) / (pow(pow(x - 153, 2) + pow(153 - y, 2), 0.5))) * 0.5 + 45 - 15);
    delay(10);
  } else if (x < 153 & y < 153 & x > 10 & y > 10) {
    servo.write(160 - ((180 / PI)*asin((153 - y) / (pow(pow(x - 153, 2) + pow(153 - y, 2), 0.5))) * 0.5 + 45 - 15));
    delay(10);
  } else if (x > 153 & y > 153) {
    servo.write((180 / PI)*abs(asin((153 - y) / (pow(pow(x - 153, 2) + pow(153 - y, 2), 0.5)))) * 0.5 + 45 - 15);
    delay(10);
  } else if (x < 153 & y > 153) {
    servo.write(160 - ((180 / PI)*asin(abs(153 - y) / (pow(pow(x - 153, 2) + pow(153 - y, 2), 0.5))) * 0.5 + 45 - 15));
    delay(10);
  }

  // --------------------------------------------------------------- DC Motor

  aux = pow(pow(x - 153, 2) + pow(153 - y, 2), 0.5);

  if (x == 255 || y == 255) {

    analogWrite(motorVelocityControl, 0);

  } else if (y < 153 && aux < 120 && y > 10) {
    analogWrite(motorVelocityControl, 0.7 * aux * 255 / 120);
    digitalWrite(motorFrontDirectionControl, HIGH);
    digitalWrite(motorBackDirectionControl, LOW);
    delay(20);
    
  } else if (y > 153 && aux < 120) {
    digitalWrite(motorFrontDirectionControl, LOW);
    digitalWrite(motorBackDirectionControl, HIGH);
    analogWrite(motorVelocityControl, 0.7 * aux * 255 / 120);
    digitalWrite(backLights, HIGH);
    delay(20);

  } else if (y > 153 && aux < 120) {
    digitalWrite(motorFrontDirectionControl, LOW);
    digitalWrite(motorBackDirectionControl, HIGH);
    analogWrite(motorVelocityControl, 0.7 * aux * 255 / 120);
    digitalWrite(backLights, HIGH);
    delay(20);
    
  } else if (y < 153 && aux < 120 && y > 10) {
    digitalWrite(motorFrontDirectionControl, HIGH);
    digitalWrite(motorBackDirectionControl, LOW);
    analogWrite(motorVelocityControl, 0.7 * aux * 255 / 120);
    delay(20);
    
  }

}


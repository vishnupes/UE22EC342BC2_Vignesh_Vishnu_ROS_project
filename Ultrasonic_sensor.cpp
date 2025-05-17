#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// LED pin
const int ledPin = 8;

long duration;
float distanceCm;
float distanceInch;

void setup() {
  lcd.begin(16, 2);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Trigger ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure duration and calculate distance
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * 0.034 / 2;
  distanceInch = distanceCm / 2.54;

  // Show distance on LCD
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(distanceCm, 1);
  lcd.print(" cm  ");

  lcd.setCursor(0, 1);
  lcd.print("Distance: ");
  lcd.print(distanceInch, 1);
  lcd.print(" inch");

  // LED logic: Turn ON if too close
  if (distanceCm < 10.0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(200);
}
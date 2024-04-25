#include <BluetoothSerial.h> // Bluetooth communication library
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <HWCDC.h>
#include <USBCDC.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <iostream>

BluetoothSerial SerialBT; // Bluetooth serial object for communication

// Define motor control pins for L298N
#define ENA_PIN 13 // Enable pin for Motor A (controls speed)
#define IN1_PIN 12 // Motor A control pin for direction
#define IN2_PIN 14 // Motor A control pin for direction
#define ENB_PIN 25 // Enable pin for Motor B (controls speed)
#define IN3_PIN 27 // Motor B control pin for direction
#define IN4_PIN 26 // Motor B control pin for direction

void stopMotors();       // Declare the stopMotors function
void accelerateMotors(); // Declare the accelerateMotors function
void reverseMotors();    // Declare the reverse motor function
void HC_SR04_loop();     // Declare the HC_SRo4_loop funtion
void checkDistance();    // Declare the check distance funtion

// Set motor speed range
int motorSpeed = 0;               // Initial motor speed
const int maxSpeed = 255;         // Maximum PWM value for ESP32 (0-255)
const int accelerationStep = 10;  // Increment speed in this step
const int delayBetweenSteps = 50; // Time delay (ms) for acceleration
const int trigPin = 5; // Trigger
const int echoPin = 18; // Echo
long duration, inches;
double cm;

void setup()
{
  Serial.begin(9600);                 // Initialize serial communication for debugging
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  SerialBT.begin("BluetoothVehicle"); // Start Bluetooth with a custom name

  // Set motor control pins as outputs
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Initially, stop both motors
  stopMotors();
}

void loop()
{
  HC_SR04_loop();
  if (SerialBT.available())
  {
    char command = SerialBT.read(); // Read incoming command from Bluetooth

    if (command == 'f')
    {                     // 'f' for forward
      accelerateMotors(); // Smooth acceleration for both motors
    }
    else if (command == 's')
    {               // 's' for stop
      stopMotors(); // Smooth deceleration for both motors
    }
    else if (command == 'r')
    {                  // 'r' for reverse
      stopMotors();    // Ensure the vehicle is stopped before reversing
      delay(500);      // Delay to allow complete stop before reversing
      reverseMotors(); // Smoothly accelerate in reverse direction
    }
  }
}

void accelerateMotors()
{
  // Set both motors to move forward
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  // Smooth acceleration to maximum speed
  while (motorSpeed < maxSpeed)
  {
    motorSpeed += accelerationStep; // Increase speed in defined step
    if (motorSpeed > maxSpeed)
    {
      motorSpeed = maxSpeed; // Keep within the maximum speed range
    }

    // Set the speed for both motors
    analogWrite(ENA_PIN, motorSpeed);
    analogWrite(ENB_PIN, motorSpeed);
    delay(delayBetweenSteps); // Wait to create a smooth acceleration
  }
}

void stopMotors()
{
  // Smooth deceleration from current speed to 0
  while (motorSpeed > 0)
  {
    motorSpeed -= accelerationStep; // Decrease speed in defined step
    if (motorSpeed < 0)
    {
      motorSpeed = 0; // Ensure speed doesn't drop below zero
    }

    // Set the speed for both motors
    analogWrite(ENA_PIN, motorSpeed);
    analogWrite(ENB_PIN, motorSpeed);
    delay(delayBetweenSteps); // Wait to create a smooth deceleration
  }

  // Fully stop the motors by setting both direction pins low
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void reverseMotors()
{ // Define the function to reverse motor direction
  // Set both motors to move backward
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  // Smoothly increase motor speed in reverse
  while (motorSpeed < maxSpeed)
  {
    motorSpeed += accelerationStep; // Increase speed in steps
    if (motorSpeed > maxSpeed)
    {
      motorSpeed = maxSpeed; // Keep within maximum PWM range
    }

    analogWrite(ENA_PIN, motorSpeed); // Set Motor A speed (reverse)
    analogWrite(ENB_PIN, motorSpeed); // Set Motor B speed (reverse)
    delay(delayBetweenSteps);         // Wait for smooth acceleration
  }
}

void HC_SR04_loop()
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  cm = (duration / 2) / 29.1;   // Divide by 29.1 or multiply by 0.0343

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(250);

  checkDistance();
}

void checkDistance()
{
  Serial.print("checkDistance");
  if (cm <= 100) // Distance from wall that will cause it to stop 
  {
    stopMotors();
  }
}
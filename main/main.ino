#include <ESP32Servo.h>
#include <math.h>
const float pi = 3.14159265;

// Initialize Revolute Joints
Servo F0;
Servo F1;
Servo F2;
Servo F3;
Servo Wrist;

// Joint and Wrist Pins
const int P0 = 23;
const int P1 = 22;
const int P2 = 21;
const int P3 = 19;
const int wr = 18;

// Number of Links, Lengths, and Joints
const int n = 4;
float L1 = 0; float L2 = 10; float L3 = 20;
float q1; float q2; float q3;

// Joint Angles
struct IKResult 
{
  long double q1;
  long double q2;
  long double q3;
};

// Initial Position
float x = 0;
float y = 0;
float z = 0;
IKResult target;

// Void Setup
void setup() 
{
  // Make DH Table
  // buildDH();
  // Attach Servos
  F0.attach(P0);
  F1.attach(P1);
  F2.attach(P2);
  F3.attach(P3);
  Wrist.attach(wr);
  // Serial Setup
  Serial.begin(115200);
  delay(1000); 
}

char decide = false;

void loop() 
{
  Serial.println("Pick / Place Decide ('1' for Yes & '0' for No):");
  // Wait for user input
  while (Serial.available() == 0) { delay(10); }
  decide = Serial.read();

  if(decide)
  {
    Serial.print("Picking : ");
    pick();
    delay(2000);
    move_top();
    delay(2000);
    Serial.println("Placing : ");
    place();
  }
}
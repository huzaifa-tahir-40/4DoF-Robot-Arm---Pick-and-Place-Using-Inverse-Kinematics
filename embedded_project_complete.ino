#include <ESP32Servo.h>
#include <math.h>
const float pi = 3.14159265;

// Initialize Revolute Joints
Servo F0;
Servo F1;
Servo F2;
Servo F3;
Servo F4;
Servo Wrist;

// Joint and Wrist Pins
int P0 = 23;
int P1 = 22;
int P2 = 21;
int P3 = 19;
int wr = 18;

// Number of Links, Lengths, and Joints
const int n = 4;
float L1 = 0; float L2 = 10; float L3 = 20;
float q1; float q2; float q3;

// Initialize Robot Parameters
// Link Twist
float alpha[n] = {0, pi/2, 0, 0};
// Link Length
float a[n] = {0, 0, L2, L3};
// Offset Distance
float d[n] = {0, L1, 0, 0};
// Joint Angles
float theta[n] = {q1, q2, q3, 0};


// Joint Angles
struct IKResult 
{
  long double q1;
  long double q2;
  long double q3;
};

// Inverse Kinematics
IKResult ik_hw(float x, float y, float z, float L1, float L2, float L3)
{
    IKResult result;

    // radial distance
    long double r = sqrt((long double)x*x + (long double)y*y);

    // compute cos(q3)
    long double cos3 = (r*r + (z - L1)*(z - L1) - L2*L2 - L3*L3) / (2.0L * L2 * L3);

    // clamp to avoid domain error
    if (cos3 > 1.0L)  cos3 = 1.0L;
    if (cos3 < -1.0L) cos3 = -1.0L;

    // compute sin(q3) safely
    long double sin3 = sqrtl(1.0L - cos3*cos3);

    // joint 3
    long double q3 = atan2l(sin3, cos3);

    // joint 1
    long double q1 = atan2l(y, x);

    // helper angle
    long double psi = atan2l(z - L1, r);

    // joint 2
    long double q2 = psi - atan2l(L3 * sin3, L2 + L3 * cos3);

    // --- keep inside 0–180 range ---
    if (q1 < 0) q1 += 360;        // bring to 0–360
    if (q1 > 180) q1 = 180;       // servo limit

    if (q2 < 0) q2 = 0;
    if (q2 > 180) q2 = 180;

    if (q3 < 0) q3 = 0;
    if (q3 > 180) q3 = 180;

    // assign
    result.q1 = q1;
    result.q2 = q2;
    result.q3 = q3;

    return result;
}

// rad2deg() Function
long double rad2deg(float angle)
{
  return angle * (180 / pi);
}

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
  // F4.attach(P4);
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

  // Read 3 floats from a single line
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

void close_jaw()
{
  F3.write(70);
}

void open_jaw()
{
  F3.write(10);
}

void pick()
{
  x = -10; y = -10; z = -10;
  target = ik_hw(x, y, z, L1, L2, L3);
  q1 = round(rad2deg(target.q1));
  q2 = round(rad2deg(target.q2));
  q3 = round(rad2deg(target.q3));
  
  // Move the Robot
  move_robot(q1, q2, q3);

  // Move Wrist :
  move_wrist(90);
  // Open Jaw :
  open_jaw();
  // Wait for 2 seconds
  delay(2000);
  // Close Jaw :
  close_jaw();
  // Wait for 2 seconds
  delay(2000);
}

void place()
{
  x = 0; y = 20; z = 10;
  target = ik_hw(x, y, z, L1, L2, L3);
  q1 = round(rad2deg(target.q1));
  q2 = round(rad2deg(target.q2));
  q3 = round(rad2deg(target.q3));

  // Move the Robot
  move_robot(q1, q2, q3);

  // Move Wrist :
  move_wrist(90);
  // Open Jaw :
  close_jaw();
  // Wait for 2 seconds
  delay(2000);
  // Close Jaw :
  open_jaw();
  // Wait for 2 seconds
  delay(2000);
}

void move_top()
{
  x = 0; y = 30; z = 0;
  target = ik_hw(x, y, z, L1, L2, L3);
  q1 = round(rad2deg(target.q1));
  q2 = round(rad2deg(target.q2));
  q3 = round(rad2deg(target.q3));
  move_wrist(0);
  move_robot(q1, q2, q3);
}

void move_wrist(float ang)
{
  Wrist.write(ang);
}

void move_robot(float q1_target, float q2_target, float q3_target)
{
  int steps = 20;

  float q1_start = F0.read();
  float q2_start = F1.read();
  float q3_start = F2.read();

  float dq1 = (q1_target - q1_start) / steps;
  float dq2 = (q2_target - q2_start) / steps;
  float dq3 = (q3_target - q3_start) / steps;

  for(int i = 0; i < steps; i++)
  {
    q1_start += dq1;
    q2_start += dq2;
    q3_start += dq3;

    F0.write(q1_start);
    F1.write(q2_start);
    F2.write(q3_start);
  }
}
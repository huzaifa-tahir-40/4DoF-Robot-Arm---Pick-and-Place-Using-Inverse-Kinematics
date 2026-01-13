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
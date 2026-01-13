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
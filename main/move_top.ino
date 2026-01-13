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
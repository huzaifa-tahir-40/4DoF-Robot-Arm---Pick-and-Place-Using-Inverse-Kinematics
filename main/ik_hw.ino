// Performs Inverse Kinematics
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
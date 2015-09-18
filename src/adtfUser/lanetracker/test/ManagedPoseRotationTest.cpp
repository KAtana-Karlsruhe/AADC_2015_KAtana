#include "ManagedPose.h"
#include <cmath>
#include <iostream>

int main( int argc, const char* argv[] )
{
    ManagedPose p(0, -1, -M_PI/2, ManagedPose::Context::IN_PIXEL);
    ManagedPose q = p.rotate(ManagedPose(0, 0, M_PI/2.0f, ManagedPose::Context::IN_PIXEL));
    cout << p.toString() << " rotated with PI/2 == " << q.toString() << endl;

    ManagedPose p_c(1, 0, 0, ManagedPose::Context::IN_CAR);
    ManagedPose q_c = p_c.rotate(ManagedPose(0, 0, M_PI/2.0f, ManagedPose::Context::IN_CAR));
    cout << p_c.toString() << " rotated with PI/2 == " << q_c.toString() << endl;

    ManagedPose center(250, 250, M_PI/4, ManagedPose::Context::IN_PIXEL);
    ManagedPose point(200, 200, 0, ManagedPose::Context::IN_PIXEL);
    ManagedPose point_r = point.rotate(center);
    cout << point.toString() << " rotated around " << center.toString() << " equals: " << endl;
    cout << point_r.toString() << endl;
}



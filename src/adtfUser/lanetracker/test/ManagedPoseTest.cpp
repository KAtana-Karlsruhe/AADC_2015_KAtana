#include "ManagedPose.h"

#include <iostream>
int main( int argc, const char* argv[] )
{
    ManagedPose p_inPixel = ManagedPose::getIMUInPixel();
    ManagedPose p_inCar(0, 0, 0, ManagedPose::Context::IN_CAR);

    ManagedPose q_inCar = ManagedPose::transformPixelToCar(p_inPixel);
    ManagedPose q_inPixel = ManagedPose::transformCarToPixel(p_inCar);
    cout << p_inPixel.toString() << " == " << q_inPixel.toString() << endl;
    cout << p_inCar.toString() << " == " << q_inCar.toString() << endl;


    ManagedPose x_c(7100, 2000, 5, ManagedPose::Context::IN_CAR);
    cout << "x: " << x_c.toString() << endl;
    ManagedPose x_p = ManagedPose::transformCarToPixel(x_c);
    cout << "x in Pixel: " << x_p.toString() << endl;
    ManagedPose x_p_c = ManagedPose::transformPixelToCar(x_p);
    cout << "x in Car: " << x_p_c.toString() << endl;

    ManagedPose x_prime = ManagedPose::transformPixelToCar(ManagedPose::transformCarToPixel(x_c));
    cout << x_c.toString() << " == " << x_prime.toString() << endl;

    ManagedPose y(20, 30, 5, ManagedPose::Context::IN_PIXEL);
    ManagedPose y_prime = ManagedPose::transformCarToPixel(ManagedPose::transformPixelToCar(y));
    cout << y.toString() << " == " << y_prime.toString() << endl;
}


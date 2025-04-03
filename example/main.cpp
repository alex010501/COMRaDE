#include <iostream>
#include <RobotArmLib.h>

int main(int argc, char** argv)
{
    std::cout << "180 degrees is " << MathAdditions::DegToRad(180) << " radians" << std::endl;
    return 0;
}
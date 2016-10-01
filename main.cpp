#include <iostream>

#include "pid.hpp"

int main(int argc, char *argv[])
{
    DiscretePID pidMotor(0.1, 0.2, 0.2);

    pidMotor.setReference(0);
    pidMotor.calculate(1);

    return 0;
}

// this script is for testing the PIDController class
// include required libraries
#include "PIDController.h"
#include <stdio.h>

// main program function
int main() {

    // creates controller object and tunes it
    PIDController pid;
    pid.tuneController(0.1, 0.01, 0.5);

    // example PID situation over 1000 iterations
    double val = 20;
    for (int i = 0; i < 1000; i++) {
        double inc = pid.calculateOutput(0, val);
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val += inc;
    }

    // normal main function endpoint
    return 0;
}
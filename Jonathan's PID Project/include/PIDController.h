#include <iostream>

// create PID controller class
class PIDController {
    
    // create all class variables
    private:
        double proportionalGain;
        double integralGain;
        double derivativeGain;
        double derivative;
        double integral;
        double lastError;

    // create all class methods
    public:

        // constructer method - used to initiate controller
        PIDController(double newProportionalGain, double newIntegralGain, double newDerivativeGain) {
          proportionalGain = newProportionalGain;
          integralGain = newIntegralGain;
          derivativeGain = newDerivativeGain;
        }

        // used to calculate pid
        double calculatePIDOutput(double targetDistance, double currentDistance) {
            double error = targetDistance - currentDistance;
            integral += error;
            derivative = error - lastError;
            lastError = error;

            std::cout << proportionalGain * error << ", " << integralGain * integral << ", " << derivativeGain * derivative << std::endl; 
            // std::cout << proportionalGain * error + integralGain * integral + derivativeGain * derivative << std::endl;
            
            
            return (proportionalGain * error + integralGain * integral + derivativeGain * derivative);
        }
};
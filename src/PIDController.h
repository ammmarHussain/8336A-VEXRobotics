// general class for PID loop and controller
class PIDController {
    
    // declaration of attributes saved in the class
    private:
        double previousError; // error in previous iteration
        double integral; // integral of error
        double derivative; // derivative of error
        double kP; // proportional gain
        double kI; // integral gain
        double kD; // derivative gain
        double output; // controller output value
    
    // declaration of functions to be used externally
    public:

        // is used to make tuning the controller easier
        void tuneController(double tuneProportionalGain, double tuneIntegralGain, double tuneDerivativeGain) {
            kP = tuneProportionalGain;
            kI = tuneIntegralGain; 
            kD = tuneDerivativeGain;
        }

        // is used to calculate the output of the controller given the set point and process variable
        double calculateOutput(double setPoint, double processVariable) {
            double error = setPoint - processVariable;
            integral += error;
            derivative = error - previousError;
            output = kP * error + kI * integral + kD * derivative;
            previousError = error;
            return output;
        }
};
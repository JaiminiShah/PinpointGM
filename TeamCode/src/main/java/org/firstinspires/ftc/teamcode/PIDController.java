package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

    public class PIDController {

        private double Kp;
        private double Ki;
        private double Kd;
        private double integralSum = 0;
        private double lastError = 0;
        public  double f=0;
        private final double ticks_in_degree=610.8/360;

        private ElapsedTime timer = new ElapsedTime(); // Timer for deltaTime calculations

        /**
         * Construct PID controller with given coefficients.
         * @param Kp Proportional coefficient
         * @param Ki Integral coefficient
         * @param Kd Derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            timer.reset(); // Initialize the timer
        }
        public PIDController(double Kp, double Ki,double Kd,double f){
            this.Kp=Kp;
            this.Ki=Ki;
            this.Kd=Kd;
            this.f=f;

        }

        /**
         * Update the PID controller output.
         * @param target Target position (desired reference)
         * @param state Current position (e.g., motor position)
         * @param tolerance Tolerance for acceptable error range
         * @return Motor power command based on PID output, or zero if within tolerance
         */
        public double update(double target, double state, double tolerance) {
            double error = target - state; // Calculate current error
            double deltaTime = timer.seconds(); // Time elapsed since last update

            // Check if error is within tolerance
            if (Math.abs(error) < tolerance) {
                integralSum = 0; // Reset integral if within tolerance
                lastError = error; // Update last error
                timer.reset(); // Reset timer for next update
                return 0; // Motor power set to 0 if within tolerance
            }

            // Proportional term
            double pTerm = Kp * error;

            // Integral term (accumulate error over time)
            integralSum += error * deltaTime;
            double iTerm = Ki * integralSum;

            // Derivative term (rate of change of error)
            double derivative = (error - lastError) / deltaTime;
            double dTerm = Kd * derivative;
            double ff=Math.cos(Math.toRadians(target/ticks_in_degree))*f;

            // PID output
            double output = pTerm + iTerm + dTerm +ff ;
            //double output1=pTerm+iTerm+dTerm;

            // Update last error and reset timer for next iteration
            lastError = error;
            timer.reset();// Reset timer after each update

            return output;
        }
    }


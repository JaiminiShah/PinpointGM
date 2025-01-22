package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@TeleOp(name="GM_Teleop2025", group="WiredWoodmen")
public class GM_Teleop2025 extends OpMode {
    DcMotorEx

            armRotator=null,
            armRotator2=null,
            armSlide = null;
  //  final double liftIncrement = 23.0;
    final double MAX_ARM_POS = 2000;
    // servo names/declarations
    Servo
            wrist = null;

    // Continuous rotation servo
    CRServo
            intake = null;;

    // ServoImplEx intake1;
    CRServoImplEx flapper;
    final double ARM_TICKS_PER_DEGREE =
            28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;
    final double liftIncrement=23.0;
    double kp=0.77
            ,ki=0.003
            ,kd=0.004,
            f=0.01;
    double GROUND_POS=0;
    double ARM_CLEAR_BARRIER=70;
    double LOW_BASKET=280;
    double HIGH_BASKET=320;
    double armPosition;
    double armPower;
    double armPower1;

    private double integralSum = 0;
    private double lastError = 0;

    private final double ticks_in_degree=610.8/360;
    double target;
    double state;
    double tolerance;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    double liftPosition = 0;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double FLAPPER_IN   = 0.7;
    final double FLAPPER_OUT  = -0.7;
    final double FLAPPER_STOP=0.0;
    double liftposincrement=23.0;

    @Override
    public void init() {
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armRotator=hardwareMap.get(DcMotorEx.class,"armRotator");
        armRotator2=hardwareMap.get(DcMotorEx.class,"armRotator2");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");

        /* Define and initialize servos.*/
        // Servo Names
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");

        //Direction?
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setDirection(DcMotor.Direction.REVERSE);
        armSlide.setDirection(DcMotor.Direction.REVERSE);
        armRotator.setPower(0);
        armRotator2.setPower(0);
        armSlide.setPower(0);
        intake.setPower(0);
        // Sets position for servos
        wrist.setPosition(.7);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armRotator).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) armRotator2).setCurrentAlert(5,CurrentUnit.AMPS);
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        telemetrylift();

        telemetry.update();
        runTime.reset();


    }
    @Override
    public void start() {
        armRotator.setTargetPosition((int)GROUND_POS);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setVelocity(400);
        armRotator2.setTargetPosition((int)GROUND_POS);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setVelocity(400);
        armSlide.setTargetPosition(0);
        // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setPower(0.0);
        telemetrylift();
    }
    public double update(double target, double state, double tolerance) {
        double error = target - state; // Calculate current error
        double deltaTime = runTime.seconds(); // Time elapsed since last update

        // Check if error is within tolerance
        if (Math.abs(error) < tolerance) {
            integralSum = 0; // Reset integral if within tolerance
            lastError = error; // Update last error
            runTime.reset(); // Reset timer for next update
            return 0; // Motor power set to 0 if within tolerance
        }

        // Proportional term
        double pTerm = kp * error;

        // Integral term (accumulate error over time)
        integralSum += error * deltaTime;
        double iTerm = ki * integralSum;

        // Derivative term (rate of change of error)
        double derivative = (error - lastError) / deltaTime;
        double dTerm = kd * derivative;
        double ff=Math.cos(Math.toRadians(target/ticks_in_degree))*f;

        // PID output
        double output=pTerm+iTerm+dTerm;
        // double output = pTerm + iTerm + dTerm +ff ;
        //double output1=pTerm+iTerm+dTerm;

        // Update last error and reset timer for next iteration
        lastError = error;
        runTime.reset(); // Reset timer after each update

        return output;
    }


    @Override
    public void loop() {
        /* Program for slides to raise up and down */
        if (gamepad2.right_bumper  && liftPosition>0) {
            liftPosition -= liftposincrement;
        }
        if(gamepad2.left_bumper && liftPosition<2300){
            liftPosition+=liftposincrement;
        }
        /*here we check to see if the lift is trying to go higher than the maximum extension.
         *if it is, we set the variable to the max.
         */
        if (liftPosition > 2300){
            liftPosition = 2300;
        }
        //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
        if (liftPosition < 0){
            liftPosition = 0;
        }

        armSlide.setTargetPosition((int) (liftPosition));
        // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armSlide).setVelocity(400);

        //  liftMotor.setPower(0.7);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Controls for intake
        if (gamepad2.left_bumper) {
            intake.setPower(0.5);
        }

        else if (gamepad2.right_bumper) {
            intake.setPower(-0.5);
        }

        else {
            intake.setPower(0);
        }

        // Controls for armRotator
        // Controls for armRotator1
                if (gamepad2.a) {
                     armPower = update(LOW_BASKET,armRotator.getCurrentPosition(),15);
                     armPower1=update(LOW_BASKET,armRotator.getCurrentPosition(),15);
                    armPosition=LOW_BASKET;

                    wrist.setPosition(.7);
               }

               else if (gamepad2.b) {
                    armPower = update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
                    armPower1=update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
                    armPosition=HIGH_BASKET;

                   wrist.setPosition(.5);
               }

                else if (gamepad2.y) {
                    armPower = update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),15);
                    armPower1=update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),15);
                    armPosition = ARM_CLEAR_BARRIER;
                    wrist.setPosition(.5);
                } else if (gamepad2.x) {
                      armPower = update(GROUND_POS,armRotator.getCurrentPosition(),15);
                      armPower1=update(GROUND_POS,armRotator.getCurrentPosition(),15);
                     armPosition = GROUND_POS;
                      wrist.setPosition(.5);
                }

                telemetry.addData("Arm Position: " , armPosition);
                telemetry.addData("Arm Encoder Counts", armRotator);
                telemetry.addData("Arm Encoder Counts 2", armRotator2);

                // this will send a telemetry message to signify robot waiting
                telemetry.addLine("I 'm Ready");
                telemetry.update();
                armRotator.setTargetPosition((int)(armPosition));
                 armRotator.setPower(armPower);


                armRotator2.setTargetPosition((int) (armPosition));
                armRotator2.setPower(armPower1);
        //       // ((DcMotorEx) armRotator2).setVelocity(200);
        //        //armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //==========================================================//
        //                        Telemetry                           //
        //==========================================================//

        telemetrylift();

        telemetryarm();




    }
    public void telemetrylift(){
        telemetry.addData("lift variable", liftPosition);
        telemetry.addData("Lift Target Position",armSlide.getTargetPosition());
        telemetry.addData("Lift motor velocity",armSlide.getVelocity());
        telemetry.addData("lift current position", armSlide.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(armSlide.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();

    }
    public void telemetryarm(){
        telemetry.addData("armRotator Target Position: ", armRotator.getTargetPosition());
        telemetry.addData("armRotator1 Encoder: ", armRotator.getCurrentPosition());
        telemetry.addData("Lift motor velocity",armRotator.getVelocity());
        telemetry.addData("Arm Motor current",armRotator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("armRotator Target Position: ", armRotator2.getTargetPosition());
        telemetry.addData("armRotator1 Encoder: ", armRotator2.getCurrentPosition());
        telemetry.addData("Lift motor velocity",armRotator2.getVelocity());
        telemetry.addData("Arm Motor current",armRotator2.getCurrent(CurrentUnit.AMPS));
    }

}

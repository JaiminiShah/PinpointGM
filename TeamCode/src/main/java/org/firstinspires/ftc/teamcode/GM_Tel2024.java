package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="GM_Tel2024", group="GreenMachine")
public class GM_Tel2024 extends OpMode {
    final double ARM_TICKS_PER_DEGREE =
            28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;
    // ARM_TICKS_PER_DEGREE is 19.794

    // 28.7 IS ARM_TICKS_PER_DEGREE
// Positions for the arms
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double GROUND_POS                = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double LOW_BASKET                = 90 * ARM_TICKS_PER_DEGREE;
    final double HIGH_BASKET               = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 300 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;
    final double MAX_ARM_POS = 1050;



    double  // Declares all double variables and their values
            speedVariable = .8;
    int speedVariable1=0;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double liftPosition = LIFT_COLLAPSED;
    double armLiftComp = 0;

    DcMotorEx armRotator = null,
            armRotator2 = null,
            armSlide = null;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();

    private final double zeroOffset = 90;

    @Override
    public void init() {
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setDirection(DcMotor.Direction.FORWARD);
        armSlide.setDirection(DcMotor.Direction.REVERSE);
        armRotator.setPower(0);
        armRotator.setTargetPosition(0);
        armRotator2.setPower(0);
        armRotator2.setTargetPosition(0);
        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setPower(0);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        //telemetrylift();
        telemetry.update();
        runTime.reset();


    }
    @Override
    public void start() {
        armRotator.setTargetPosition(0);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setPower(0);
        armRotator2.setTargetPosition(0);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setPower(0);

        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetryarm();
        telemetrylift();

    }

    @Override
    public void loop() {
        armPositionFudgeFactor=FUDGE_FACTOR*(gamepad1.right_trigger +(-gamepad1.left_trigger));
        //==========================================================\\
        //                        GamePad Two                       \\
        //==========================================================\\

        // Controls for intake
       /* if (gamepad2.left_bumper) {
            intake.setPower(0.5);
        }

        else if (gamepad2.right_bumper) {
            intake.setPower(-0.5);
        }*/

        // Controls for arm slide
        if (gamepad2.right_trigger > .1) {
            liftPosition += 2800 * cycletime;
        }

        else if (gamepad2.left_trigger > .1) {
            liftPosition -= 2800 * cycletime;
        }

        // Makes sure the lift does not go beyond parameters
        if (liftPosition > MAX_ARM_POS){
            liftPosition = MAX_ARM_POS;
        }

        else if (liftPosition < 0){
            liftPosition = 0;
        }

        armSlide.setTargetPosition((int) (liftPosition));

        ((DcMotorEx) armSlide).setVelocity(200);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Controls for armRotator
        if (gamepad2.a) {
            armPosition = GROUND_POS;
            //wrist.setPosition(.3);
        }

        else if (gamepad2.b) {
            armPosition = LOW_BASKET;
            //wrist.setPosition(.5);
        }

        else if (gamepad2.y) {
            armPosition = HIGH_BASKET;
          //  wrist.setPosition(.5);

        }
        if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
            armLiftComp = (0.25568 * liftPosition);
        }
        else{
            armLiftComp = 0;
        }
        armRotator.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
        armRotator2.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
        ((DcMotorEx) armRotator).setVelocity(2100);
        ((DcMotorEx) armRotator2).setVelocity(2100);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        looptime = getRuntime();
        cycletime = looptime-oldtime;
        oldtime = looptime;

        //==========================================================\\
        //                        Telemetry                         \\
        //==========================================================\\

        telemetrylift();
        telemetryarm();

    }
    public void telemetrylift(){
        telemetry.addData("lift variable", MAX_ARM_POS);
        telemetry.addData("Lift Target Position",armSlide.getTargetPosition());
        telemetry.addData("lift current position", armSlide.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(armSlide.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();

    }
    public void telemetryarm(){
        telemetry.addData("Rotator Target Position: ", armRotator.getTargetPosition());
        telemetry.addData("Rotator2 target Position",armRotator2.getTargetPosition());
        telemetry.addData("Rotator Target Position: ",armRotator.getTargetPosition());
        telemetry.addData("Rotator2 Target Position:",armRotator2.getTargetPosition());
        telemetry.update();
    }
    @Override
    public void stop() {
        // Sets all motors to zero power except Arms to keep pos

        armSlide.setPower(0);
        armRotator.setPower(0);
        armRotator2.setPower(0);
    }

    //double armPosition = 0;
}

package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.PinpointDrive;


//import org.firstinspires.ftc.teamcode.PIDCoefficients;


@Autonomous(name = "GM_AutoBRL2024", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class GM_AutoBRL2024 extends LinearOpMode {
    public static String TEAM_NAME = "GreenMachine"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 8791; //TODO: Enter team Number
    double kp = 0.77,
            ki = 0.000,
            kd = 0.004,
            f = 0.01;
    int GROUND_POS = 0;
    int ARM_CLEAR_BARRIER = 50;
    int LOW_BASKET = 300;
    int HIGH_BASKET = 500;
    double armPosition;
    PIDController pid1 = new PIDController(kp, ki, kd, f);
    double armPower, armPower1;
    double time = 0;


   /* final double ARM_TICKS_PER_DEGREE =
            28 // Number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100/20 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0;
    // Positions for the arms
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double GROUND_POS                = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double LOW_BASKET                = 50.9337861 * ARM_TICKS_PER_DEGREE;
    final double HIGH_BASKET               = 101.867572 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;*/

    final double LIFT_COLLAPSED = 0;
    final double LIFT_SCORING_IN_LOW_BASKET = 1000;
    final double LIFT_SCORING_IN_HIGH_BASKET = 700;


    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUELEFT,
        REDLEFT
    }

    public static GM_AutoBL2024.START_POSITION startPosition;
    DcMotorEx armRotator = null,
            armRotator2 = null,
            armSlide = null;
    ServoImplEx wrist = null;
    CRServoImplEx intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Key Pad input to selecting Starting Position of robot
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator.setDirection(DcMotor.Direction.REVERSE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        armRotator2.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armSlide.setDirection(DcMotor.Direction.REVERSE);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        // wristUp();
        intake = hardwareMap.get(CRServoImplEx.class, "intake");

        while (!isStopRequested()) {
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:", "");
            telemetry.addData("    Left   ", "(X / ▢)");
            telemetry.addData("    Right ", "(Y / Δ)");

            if (gamepad1.x) {
                startPosition = GM_AutoBL2024.START_POSITION.BLUELEFT; //Blue Left
                break;
            }
            if (gamepad1.y) {
                startPosition = GM_AutoBL2024.START_POSITION.REDLEFT; //Red
                break;
            }
            telemetry.update();

        }
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.update();

        waitForStart();
        // Game Play Button is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();

        }


    }// end runOpMode()

    public void moveArm(double position) {
        armPower = pid1.update(position, armRotator.getCurrentPosition(), 15);
        armPower1 = pid1.update(position, armRotator2.getCurrentPosition(), 15);
        armRotator.setPower(armPower);
        armRotator2.setPower(armPower);
        armPosition = position;

    }

    public void wristUp() {
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist.setPosition(0.5);

    }

    public void wristDown() {
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist.setPosition(1.0);

    }

    public void intakeF() {
        intake.setPower(0.5);
    }

    public void intakeR() {
        intake.setPower(-0.5);
    }

    public void slideUp(double uposition) {
        armSlide.setTargetPosition((int) uposition);
        armSlide.setPower(0.7);
        armSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void runAutonoumousMode() {
        //Auto Blue Left Positions - Samples
        Pose2d initPose = new Pose2d(31.2, 61.7, Math.toRadians(0)); // Starting Pose

        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        if (startPosition == GM_AutoBL2024.START_POSITION.BLUELEFT) {

            //Move robot to netZone with preloaded sample ready to drop in basket

            Actions.runBlocking(
                    drive.actionBuilder(initPose)

                            .splineToConstantHeading(new Vector2d(55, 55), Math.toRadians(0))
                            .waitSeconds(0.4)
                            .splineToLinearHeading(new Pose2d(35, 12, 0), Math.toRadians(270))
                            .splineToConstantHeading(new Vector2d(38, 12), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(55, 55, 0), Math.toRadians(0))
                            .waitSeconds(.4)
                            .splineToLinearHeading(new Pose2d(48, 12, 0), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(57, 55, 0), Math.toRadians(0))
                            .waitSeconds(.4)
                            .splineToLinearHeading(new Pose2d(34, -5, 0), Math.toRadians(0))


                            .build());
        }

        /*    safeWaitSeconds(1);
            telemetry.addLine("Move robot to netZone");
            telemetry.update();
            //Add code to drop sample in basket
            //moveArm(LOW_BASKET);
            //  slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristDown();
            intakeF();
            intakeR();
            safeWaitSeconds(3);*/

        /*    safeWaitSeconds(2);
            slideUp(LIFT_COLLAPSED);
            //intake.setPower(0.0);
            telemetry.addLine("Drop sample in basket");
            telemetry.update();*/

        //Move robot to pick yellow sample one
           /* Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(yellowSampleOne.position, yellowSampleOne.heading)

                            .splineToSplineHeading(new Pose2d(new Vector2d(38,11),Math.toRadians(0)),Math.toRadians(0))
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to pick yellow sample one");
            telemetry.update();
            //Add code to pick up yellow sample
         /*   moveArm(ARM_CLEAR_BARRIER);
            slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristUp();
            intakeF();

            safeWaitSeconds(1);

            telemetry.addLine("Pick up yellow sample");
            telemetry.update();

            //Move robot to net zone to drop sample
            Actions.runBlocking(
                    drive.actionBuilder(yellowSampleOne)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone to drop sample");
            telemetry.update();
            //Add code to drop sample in bucket
            moveArm(LOW_BASKET);
            slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristDown();
            intakeR();

            safeWaitSeconds(2);
            slideUp(LIFT_COLLAPSED);
            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to yellow sample two
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(yellowSampleTwo.position, yellowSampleTwo.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to yellow sample two");
            telemetry.update();
            //Add code to pick up yellow sample
            moveArm(ARM_CLEAR_BARRIER);
            wristUp();
            intakeF();

            safeWaitSeconds(1);
            telemetry.addLine("Pick up yellow sample");
            telemetry.update();

            //Move robot to net zone
            Actions.runBlocking(
                    drive.actionBuilder(yellowSampleTwo)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone");
            telemetry.update();

            //Add code to drop sample in bucket
            moveArm(LOW_BASKET);
            slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristDown();
            intakeR();
            safeWaitSeconds(1);
            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to submersible parking
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(preSubmersiblePark.position, preSubmersiblePark.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to preSubmersible parking");
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(preSubmersiblePark)
                            .strafeToLinearHeading(submersiblePark.position, submersiblePark.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("hitting bottom rung");
            telemetry.update();
            //add code to hit bottom rung
            moveArm(ARM_CLEAR_BARRIER);
            slideUp(LIFT_COLLAPSED);*/


        //else { // REDLEFT
        // Move robot to netZone with preloaded sample ready to drop in basket
          /*  Actions.runBlocking(
                    drive.actionBuilder(redinitPose)
                            .strafeToLinearHeading(rednetzone.position, rednetzone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to netZone");
            telemetry.update();
            //Add code to drop sample in basket
            moveArm(LOW_BASKET);
            slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristDown();
            intakeR();

            safeWaitSeconds(2);
            slideUp(LIFT_COLLAPSED);
            //intake.setPower(0.0);
            telemetry.addLine("Drop sample in basket");
            telemetry.update();

            //Move robot to pick yellow sample one
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(redyellowone.position, redyellowone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to pick yellow sample one");
            telemetry.update();
            //Add code to pick up yellow sample
            moveArm(ARM_CLEAR_BARRIER);
            wristUp();
            intakeF();

            safeWaitSeconds(1);

            telemetry.addLine("Pick up yellow sample");
            telemetry.update();

            //Move robot to net zone to drop sample
            Actions.runBlocking(
                    drive.actionBuilder(redyellowone)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone to drop sample");
            telemetry.update();
            //Add code to drop sample in bucket
            moveArm(LOW_BASKET);
            slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristDown();
            intakeR();

            safeWaitSeconds(2);
            slideUp(LIFT_COLLAPSED);
            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to yellow sample two
            Actions.runBlocking(
                    drive.actionBuilder(rednetzone)
                            .strafeToLinearHeading(redyellowtwo.position, redyellowtwo.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to yellow sample two");
            telemetry.update();
            //Add code to pick up yellow sample
            moveArm(ARM_CLEAR_BARRIER);
            wristUp();
            intakeF();

            safeWaitSeconds(1);
            telemetry.addLine("Pick up yellow sample");
            telemetry.update();

            //Move robot to net zone
            Actions.runBlocking(
                    drive.actionBuilder(redyellowtwo)
                            .strafeToLinearHeading(rednetzone.position, rednetzone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone");
            telemetry.update();

            //Add code to drop sample in bucket
            moveArm(LOW_BASKET);
            slideUp(LIFT_SCORING_IN_LOW_BASKET);
            wristDown();
            intakeR();
            safeWaitSeconds(1);
            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to submersible parking
            Actions.runBlocking(
                    drive.actionBuilder(rednetzone)
                            .strafeToLinearHeading(colorSampleOne.position, colorSampleOne.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to preSubmersible parking");
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(colorSampleOne)
                            .strafeToLinearHeading(observationPark.position,observationPark.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("hitting bottom rung");
            telemetry.update();
            //add code to hit bottom rung
            moveArm(ARM_CLEAR_BARRIER);
            slideUp(LIFT_COLLAPSED);

        }*/


    }
   // method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    }  // end class



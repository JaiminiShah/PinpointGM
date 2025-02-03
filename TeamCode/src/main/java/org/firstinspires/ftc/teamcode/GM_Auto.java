package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Vector;


//import org.firstinspires.ftc.teamcode.PIDCoefficients;


@Autonomous(name = "GM_Auto", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class GM_Auto extends LinearOpMode {
    public static String TEAM_NAME = "GreenMachine"; //TODO: Enter team Nam
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

    final double LIFT_COLLAPSED = 0;
    final double LIFT_SCORING_IN_LOW_BASKET = 1000;
    final double LIFT_SCORING_IN_HIGH_BASKET = 700;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUELEFT,
        REDLEFT
    }

    public static START_POSITION startPosition;
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
                startPosition = START_POSITION.BLUELEFT; //Blue Left
                break;
            }
            if (gamepad1.y) {
                startPosition = START_POSITION.REDLEFT; //Red
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
//Build parking trajectory based on last detected target by vis
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
        //       wrist.setPwmRange(new PwmControl.PwmRange(500,2500));
        wrist.setPosition(0.2);

    }

    public void wristDown() {
        //     wrist.setPwmRange(new PwmControl.PwmRange(500,2500));
        wrist.setPosition(.3);

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

        Pose2d initPose = new Pose2d(35, 63, Math.toRadians(0)); // Starting Pose
        Pose2d Pose1= new Pose2d(56,60, Math.toRadians(0));
        Pose2d Pose2= new Pose2d(31,29, Math.toRadians(0));



        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        if (startPosition == START_POSITION.BLUELEFT) {

            wristUp();
            moveArm(100);

            waitSeconds(2);

            Actions.runBlocking(
                    drive.actionBuilder(initPose)
                            .strafeTo(new Vector2d(56, 60))
                            .build()
            );

            safeWaitSeconds(1);

            wristDown();
            intakeF();
            intakeR();
            safeWaitSeconds(1);
            wristUp();
            waitSeconds(1);

            Actions.runBlocking(
                    drive.actionBuilder(Pose1)
                            .strafeTo(new Vector2d(31, 29))
                            .build());

        //    moveArm(150);
        //    wristDown();
        //    safeWaitSeconds(1);
        //    intakeF();
        //    safeWaitSeconds(1);

            Actions.runBlocking(
                    drive.actionBuilder(Pose2)

         //                   .splineTo(new Vector2d(38,61), Math.toRadians(0))
         //                   .splineTo(new Vector2d(38,12), Math.toRadians(0))
                            .strafeToConstantHeading(new Vector2d(44,30))
                            .splineToConstantHeading(new Vector2d(56,63), Math.toRadians(0))
                            .strafeToConstantHeading(new Vector2d(44,58))

                            .build());
                            /*.forward(30)
                            .strafeToLinearHeading(new Vector2d(47, 10), Math.toRadians(0))
                            //.back(55)
                            //.forward(55)
                            .strafeTo(new Vector2d(57,10))
                            //.back(55)
                            //.forward(55)
                            .strafeTo(new Vector2d(63,10))
                            //.back(55)
                            .strafeTo(new Vector2d(27, 0))*/


        }

    }

    private void waitSeconds(int i) {
    }

    private void safeWaitSeconds(int i) {
    }
}
package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Vector;


@Autonomous(name = "GM_Auto1", group = "exercises")

public class GM_Auto1 extends LinearOpMode {
    DcMotorEx armRotator = null,
            armRotator2 = null,
            armSlide = null;
    ServoImplEx wrist = null;
    CRServoImplEx intake = null;
    int GROUND_POS = 0;
    int ARM_CLEAR_BARRIER = 50;
    int LOW_BASKET = 300;
    int HIGH_BASKET = 500;

    public static String TEAM_NAME = "GreenMachine"; //TODO: Enter team Nam
    public static int TEAM_NUMBER = 8791;
    double kp = 0.77,
            ki = 0.000,
            kd = 0.004,
            f = 0.01;
    PIDController pid1=new PIDController(kp,ki,kd);
    double armPower, armPower1;
    public int armTarget;
    public int slidepos;
    private final double ticks_in_degree=610.8/360;

    //Define and declare Robot Starting Locations

    public enum START_POSITION {
        BLUELEFT,
        REDLEFT
    }

    public static START_POSITION startPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        Arm arm1= new Arm(hardwareMap);
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armSlide.setDirection(DcMotor.Direction.REVERSE);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
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


    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(35, 63, Math.toRadians(0)); // Starting Pose
        Pose2d Pose1 = new Pose2d(56, 60, Math.toRadians(0));
        Pose2d Pose2 = new Pose2d(31, 29, Math.toRadians(0));
        Arm arm1=new Arm(hardwareMap);
        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);
        if (startPosition == START_POSITION.BLUELEFT) {
            TrajectoryActionBuilder dropoffPreload=drive.actionBuilder(initPose)
                    .splineToConstantHeading(new Vector2d(53,52),Math.toRadians(0))
                    .waitSeconds(0.01);
            TrajectoryActionBuilder toFirstSample = drive.actionBuilder(new Pose2d(53,52,Math.toRadians(0)))
                    .setTangent(Math.toRadians(224.65))
                    .splineToSplineHeading(new Pose2d(36, 25,0), Math.toRadians(270))
                    .waitSeconds(.1);
            TrajectoryActionBuilder dropOffFirst = drive.actionBuilder(new Pose2d(48.1,38.5,Math.toRadians(90)))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(59, 59,0), Math.toRadians(45))
                    .waitSeconds(.2);
            // Move to second sample while resetting output box and retracting slides
            TrajectoryActionBuilder toSecondSample = drive.actionBuilder(new Pose2d(52,53,Math.toRadians(49.5)))
                    .setTangent(Math.toRadians(225))
                    .splineToSplineHeading(new Pose2d(46, 25, 0),Math.toRadians(270))
                    .waitSeconds(.1);
            // Move to basket the third time and deposit second sample
            TrajectoryActionBuilder dropOffSecond = drive.actionBuilder(new Pose2d(64,40.5,Math.toRadians(99)))
                    .setTangent(90)
                    .splineToSplineHeading(new Pose2d(59, 59,0),Math.toRadians(45))
                    .waitSeconds(.1);
            // Move to third sample while resetting output box and retracting slides
            TrajectoryActionBuilder toThirdSample = drive.actionBuilder(new Pose2d(57,49.5,Math.toRadians(48)))
                    .setTangent(225)
                    .splineToSplineHeading(new Pose2d(56, 25, 0),Math.toRadians(270))
                    .waitSeconds(.1);
            // Move to basket the fourth time and deposit third sample
            TrajectoryActionBuilder dropOffThird = drive.actionBuilder(new Pose2d(63.2,41.5,Math.toRadians(135)))
                    .setTangent(90)
                    .splineToSplineHeading(new Pose2d(59, 59,0), Math.toRadians(45))
                    .waitSeconds(.2);
            //Park
            TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(60,48.5,Math.toRadians(48)))
                    .setReversed(false)
                    .splineTo(new Vector2d(30,11),Math.toRadians(0))
                    .waitSeconds(.1);


            // Initialization
            Actions.runBlocking(
                    new SequentialAction(

                    ) );


            Actions.runBlocking(
                    new ParallelAction(
                            arm1.UpdatePID(),
                            new SequentialAction(
                                    arm1.moveToPosition(LOW_BASKET),
                                    moveSlide(700),
                                    dropoffPreload.build()
                                    //  new ParallelAction(
                                    // wristDown(),
                                    // intakeEject())


                            )

                    ));

        }
    }
    public class Arm {
        DcMotorEx armRotator=null,
                armRotator2=null;

        public Arm(HardwareMap hardwareMap) {
            armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
            armRotator.setDirection(DcMotor.Direction.REVERSE);
            armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
            armRotator2.setDirection(DcMotor.Direction.FORWARD);
            armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRotator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class MoveArm implements Action {
            int targetPosition;

            private boolean initialized = false;

         /*   public MoveArm(int targetPosition) {
                this.targetPosition = targetPosition;
                new SleepAction(1);

            }*/

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pid1.setPID(kp,ki,kd);
                int armpos1 = armRotator.getCurrentPosition();
                int armpos2 = armRotator2.getCurrentPosition();
                double armPID = pid1.calculate(armpos1,armTarget);
                double armPID1 = pid1.calculate(armpos2, armTarget);
                double armFF = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
                double armFF1 = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
                double armPower = armPID + armFF;
                double armPower1 = armPID1 + armFF1;
                armRotator.setTargetPosition(armTarget);
                armRotator2.setTargetPosition(armTarget);
                armRotator.setPower(armPower);
                armRotator2.setPower(armPower1);
                packet.put("Arm1 Position", armRotator.getCurrentPosition());
                packet.put("Arm2 Position", armRotator2.getCurrentPosition());
                return true;
            }
        }
        public Action UpdatePID(){
            return new MoveArm();
        }
        public Action moveToPosition(int position) {
            // return new MoveArm(position);
            return new InstantAction(()->armTarget=position);
        }
    }
    public class moveSlide implements Action{
        int targetSlide;
        public moveSlide(int targetSlide){
            this.targetSlide=targetSlide;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slidepos=targetSlide;
            armSlide.setTargetPosition(targetSlide);
            armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlide.setPower(0.7);
            return false;
        }
    }
    public Action moveSlide(int targetSlide){
        return new moveSlide(targetSlide);
    }
    public Action wristUp(){
        return new InstantAction(()->wrist.setPosition(0.2));
    }
    public Action wristDown(){
        return new InstantAction(()->wrist.setPosition(0.3));
    }
    public Action intakeGrab(){
        return new InstantAction(()->intake.setPower(0.5));
    }
    public Action intakeEject(){
        return new InstantAction(()->intake.setPower(-0.5));
    }

}

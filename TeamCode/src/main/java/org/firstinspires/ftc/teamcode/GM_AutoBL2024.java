package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


//import org.firstinspires.ftc.teamcode.PIDCoefficients;


@Autonomous(name = "GM_AutoBL2024", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class GM_AutoBL2024 extends LinearOpMode {
    public static String TEAM_NAME = "GreenMachine"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 8791; //TODO: Enter team Number
    final double ARM_TICKS_PER_DEGREE =
            25 // Number of encoder ticks per rotation of the bare motor
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
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;




    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        LEFT,
        RIGHT
    }
    public static START_POSITION startPosition;
    DcMotorEx armRotator=null,
              armRotator2=null,
              armSlide=null;
    ServoImplEx wrist=null;
    CRServoImplEx intake=null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Key Pad input to selecting Starting Position of robot
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator.setDirection(DcMotor.Direction.REVERSE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        armRotator2.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armSlide.setDirection(DcMotor.Direction.FORWARD);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Left   ", "(X / ▢)");
            telemetry.addData("    Right ", "(Y / Δ)");

            if(gamepad1.x){
                startPosition = START_POSITION.LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.RIGHT;
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

        }


    }// end runOpMode()
    public void moveArm(double position) {
        armRotator.setTargetPosition((int) (position));
        armRotator.setVelocity(700);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator2.setTargetPosition((int) (position));
    }
    public void wristUp(){
        wrist.setPwmRange(new PwmControl.PwmRange(500,2500));
        wrist.setPosition(0.5);

    }
    public void wristDown(){
       wrist.setPwmRange(new PwmControl.PwmRange(500,2500));
       wrist.setPosition(1.0);

    }
    public void intakeF(){
        intake.setPower(1);
    }
    public void intakeR(){
        intake.setPower(-1);

    }
    public void slideUp(double uposition){
        armSlide.setTargetPosition((int)uposition);
        armSlide.setPower(0.7);
        armSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    public void runAutonoumousMode() {
        //Auto Left Positions - Samples
        Pose2d initPose = new Pose2d(42, 63, Math.toRadians(0)); // Starting Pose
        Pose2d submersibleSpecimen = new Pose2d(28,-1,Math.toRadians(0) );
        Pose2d netZone = new Pose2d(9  ,15,Math.toRadians(-45));
        Pose2d yellowSampleOne = new Pose2d(18,12,Math.toRadians(-14));
        Pose2d yellowSampleTwo = new Pose2d(18,18,Math.toRadians(1));
        Pose2d preSubmersiblePark = new Pose2d(58,11,Math.toRadians(0));
        Pose2d submersiblePark = new Pose2d(59,-15,Math.toRadians(90));

        //Auto Right Positions - Specimens
        Pose2d observationZone = new Pose2d(8,-37,Math.toRadians(0));
        Pose2d specimenPickup = new Pose2d(3,-30,Math.toRadians(0));
        Pose2d preColorSampleOne = new Pose2d(28,-27,Math.toRadians(0));
        Pose2d prePushColorSampleOne = new Pose2d(51,-27,Math.toRadians(0));
        Pose2d colorSampleOne = new Pose2d(51,-40,Math.toRadians(0));
        Pose2d observationPark = new Pose2d(5,-30,Math.toRadians(0));

        double waitSecondsBeforeDrop = 0;
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        if (startPosition == START_POSITION.LEFT) {

            //Move robot to netZone with preloaded sample ready to drop in basket
            Actions.runBlocking(
                    drive.actionBuilder(initPose)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to netZone");
            telemetry.update();
            //Add code to drop sample in basket
            moveArm(HIGH_BASKET);
            slideUp(LIFT_SCORING_IN_HIGH_BASKET);
            wristDown();
            intakeR();

            safeWaitSeconds(1);
            //intake.setPower(0.0);
            telemetry.addLine("Drop sample in basket");
            telemetry.update();

            //Move robot to pick yellow sample one
            Actions.runBlocking(
                    drive.actionBuilder(netZone)
                            .strafeToLinearHeading(yellowSampleOne.position, yellowSampleOne.heading)
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
                    drive.actionBuilder(yellowSampleOne)
                            .strafeToLinearHeading(netZone.position, netZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to net zone to drop sample");
            telemetry.update();
            //Add code to drop sample in bucket
            moveArm(HIGH_BASKET);
            wristDown();
            intakeR();

            safeWaitSeconds(1);
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
            moveArm(HIGH_BASKET);
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
            slideUp(LIFT_COLLAPSED);


        } else { // RIGHT

            //Move robot with preloaded specimen to submersible to place specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(submersibleSpecimen.position)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible to place specimen");
            telemetry.update();
            //add code to hang specimen
            safeWaitSeconds(1);
            telemetry.addLine("Hang specimen");
            telemetry.update();



            //Move robot to color sample 1 (pushing - can change for your liking)
            Actions.runBlocking(
                    drive.actionBuilder(submersibleSpecimen)
                            .strafeToLinearHeading(preColorSampleOne.position, preColorSampleOne.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to preColor Sample");
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(preColorSampleOne)
                            .strafeToLinearHeading(prePushColorSampleOne.position, prePushColorSampleOne.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to preColor Sample");
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(prePushColorSampleOne)
                            .strafeToLinearHeading(colorSampleOne.position, colorSampleOne.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to Color Sample");
            telemetry.update();

            Actions.runBlocking(
                    drive.actionBuilder(colorSampleOne)
                            .strafeToLinearHeading(observationZone.position, observationZone.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot and pushing sample to observation zone");
            telemetry.update();

            //Move robot to pickup specimen
            Actions.runBlocking(
                    drive.actionBuilder(observationZone)
                            .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot specimen pickup ");
            telemetry.update();
            //add code to pick up specimen
            safeWaitSeconds(1);
            telemetry.addLine("Pick up specimen");
            telemetry.update();

            //Move robot to  submersible specimen
            Actions.runBlocking(
                    drive.actionBuilder(specimenPickup)
                            .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible specimen");
            telemetry.update();
            //add code to hang specimen
            safeWaitSeconds(1);
            telemetry.addLine("Hang specimen");
            telemetry.update();

            //Move robot to pickup specimen
            Actions.runBlocking(
                    drive.actionBuilder(submersibleSpecimen)
                            .strafeToLinearHeading(specimenPickup.position, specimenPickup.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot specimen pickup ");
            telemetry.update();
            //add code to pick up specimen
            safeWaitSeconds(1);
            telemetry.addLine("Pick up specimen");
            telemetry.update();



            //Move robot to submersible
            Actions.runBlocking(
                    drive.actionBuilder(specimenPickup)
                            .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible");
            telemetry.update();

            //add code to hang specimen
            safeWaitSeconds(1);
            telemetry.addLine("Hang specimen");
            telemetry.update();


            //Move robot to observation parking
            Actions.runBlocking(
                    drive.actionBuilder(submersibleSpecimen)
                            .strafeToLinearHeading(observationPark.position, observationPark.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to observation parking");
            telemetry.update();
        }

    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}   // end class



package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.lang.Math;

@Autonomous(name = "GM_AutoBR2024",group="Autonomous")

public class GM_AutoBR2024 extends LinearOpMode{
    final double ARM_TICKS_PER_DEGREE =
            25 // Number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100/20 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

// 17.67 IS ARM_TICKS_PER_DEGREE :)


    // Positions for the arms
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double GROUND_POS                = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double LOW_BASKET                = 50.9337861 * ARM_TICKS_PER_DEGREE;
    final double HIGH_BASKET               = 101.867572 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    final double LiftUpperbasket=1800;
    final double LiftLowerbasket=1500;
    final double LiftGrabSample=500;

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

            public MoveArm(int targetPosition) {
                this.targetPosition = targetPosition;
                new SleepAction(1);

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    armRotator.setTargetPosition(targetPosition);
                    armRotator.setPower(0.7);
                    armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRotator2.setTargetPosition(targetPosition);
                    armRotator2.setPower(0.7);
                    armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                packet.put("Arm1 Position", armRotator.getCurrentPosition());
                packet.put("Arm2 Position", armRotator2.getCurrentPosition());
                return false;

            }
        }

        public Action moveToPosition(int position) {

                return new MoveArm(position);
        }
    }
    public class Lift {
        DcMotorEx armSlide;

        public Lift(HardwareMap hardwareMap) {
            armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
            armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUpperBasket implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armSlide.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = armSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2000.0) {
                    //            // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    armSlide.setPower(0);
                    return false;
                }
                //        // overall, the action powers the lift until it surpasses
                //        // 3000 encoder ticks, then powers it off
            }

        }

        public Action liftUpperBasket() {
            return new LiftUpperBasket();
        }

        public class LiftLowerBasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armSlide.setPower(0.8);
                    initialized = true;
                }

                double pos = armSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1500.0) {
                    return true;
                } else {
                    armSlide.setPower(0);
                    return false;
                }
            }


        }

        public Action liftLowerBasket() {
            return new LiftLowerBasket();
        }


    }

        public class Wrist {
             ServoImplEx wrist;

            public Wrist(HardwareMap hardwareMap) {
                wrist = hardwareMap.get(ServoImplEx.class, "wrist");

            }

            public class UpWrist implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    wrist.setPosition(0.7);
                    return false;
                }
            }

            public Action upWrist() {
                return new UpWrist();
            }


            public class DownWrist implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    wrist.setPosition(0.55);
                    return false;
                }
            }

            public Action downWrist() {
                return new DownWrist();
            }
        }
        public class Claw {
              private CRServoImplEx Intake;
              public Claw(HardwareMap hardwareMap){
                  Intake =hardwareMap.get(CRServoImplEx.class,"Intake");

              }
              public class GrabIntake implements Action{


                  @Override
                  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                      Intake.setPower(0.7);
                      return false;
                  }
              }
              public Action grabIntake(){
                  return new GrabIntake();
              }
              public class DepositIntake implements Action{

                  @Override
                  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                      Intake.setPower(-0.7);
                      return false;
                  }
              }
              public Action depositIntake(){
                  return new DepositIntake();
              }
              public class StopIntake implements Action {

                  @Override
                  public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                      Intake.setPower(0);
                      return false;
                  }
              }
              public Action stopIntake(){
                  return new StopIntake();
              }
        }




            @Override
            public void runOpMode() {
                Pose2d initialPose = new Pose2d(31.2, 61.7, Math.toRadians(270));
                PinpointDrive drive =new PinpointDrive(hardwareMap,initialPose);
                Arm arm =new Arm(hardwareMap);
                Lift lift=new Lift(hardwareMap);
                Wrist wrist1=new Wrist(hardwareMap);
                Claw Intake1 =new Claw(hardwareMap);
                Action fourSampleAuto1= drive.actionBuilder(initialPose)
                        //Score sample preload
                        .strafeToLinearHeading(new Vector2d(58,53),Math.toRadians(225))
                        .build();
                Action fourSampleAuto2= drive.actionBuilder(new Pose2d(58,53, Math.toRadians(225)))
                        // Go to sample zone 1
                        .strafeToLinearHeading(new Vector2d(53,51), Math.toRadians(270))
                        .build();
                Action fourSampleAuto3 = drive.actionBuilder(new Pose2d(53,51, Math.toRadians(270)))
                        // Score bucket
                        .strafeToLinearHeading(new Vector2d(55,55), Math.toRadians(225))
                        .build();
                Action fourSampleAuto4 = drive.actionBuilder(new Pose2d(55,55, Math.toRadians(225)))
                        // Sample zone 2
                        .strafeToLinearHeading(new Vector2d(63,51), Math.toRadians(273))
                        .build();
                Action fourSampleAuto5 = drive.actionBuilder(new Pose2d(63,51, Math.toRadians(273)))
                        // Score bucket
                        .strafeToLinearHeading(new Vector2d(55,55), Math.toRadians(225))
                        .build();
                Action fourSampleAuto6 = drive.actionBuilder(new Pose2d(55,55, Math.toRadians(225)))
                        // sample zone 3
                        .strafeToLinearHeading(new Vector2d(67,51), Math.toRadians(285))
                        .build();
                Action fourSampleAuto7 = drive.actionBuilder(new Pose2d(-67,-49.3, Math.toRadians(285)))
                        // turn and score bucket
                        .strafeToLinearHeading(new Vector2d(54.5,53.5), Math.toRadians(225))
                        .build();
                Action fourSampleAuto8= drive.actionBuilder(new Pose2d(54.5,53.5, Math.toRadians(225)))
                        // park
                        .strafeToLinearHeading(new Vector2d(44,6), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(24.2,6), Math.toRadians(0))

                        .build();
                Actions.runBlocking(
                        new SequentialAction(
                        new ParallelAction(arm.moveToPosition((int)HIGH_BASKET),
                                lift.liftUpperBasket()),

                                fourSampleAuto1,
                                new ParallelAction(
                                wrist1.downWrist(),
                                        Intake1.depositIntake()),
                                new SleepAction(1),
                                new ParallelAction(
                                Intake1.stopIntake(),
                                fourSampleAuto2),
                                arm.moveToPosition((int)ARM_CLEAR_BARRIER),

                                new ParallelAction(
                                     wrist1.upWrist(),
                                        Intake1.grabIntake()
                                ),
                                new SleepAction(1),
                                new ParallelAction(
                                   Intake1.stopIntake(),

                                   arm.moveToPosition((int)HIGH_BASKET)
                                ),
                                fourSampleAuto3,
                                new ParallelAction(
                                        wrist1.downWrist(),
                                        Intake1.depositIntake()

                                ),
                                new SleepAction(1),
                                fourSampleAuto4,
                                arm.moveToPosition((int)ARM_CLEAR_BARRIER),
                                new ParallelAction(
                                        wrist1.upWrist(),
                                        Intake1.grabIntake()

                                ),
                                new SleepAction(1),
                                new ParallelAction(
                                        Intake1.stopIntake(),

                                        arm.moveToPosition((int)HIGH_BASKET)
                                ),
                                fourSampleAuto5,
                                new ParallelAction(
                                        wrist1.downWrist(),
                                        Intake1.depositIntake()

                                ),
                                new SleepAction(1),
                                fourSampleAuto6,
                                arm.moveToPosition((int)ARM_CLEAR_BARRIER),
                                new ParallelAction(
                                        wrist1.upWrist(),
                                        Intake1.grabIntake()

                                ),
                                new SleepAction(1),
                                new ParallelAction(
                                        Intake1.stopIntake(),

                                        arm.moveToPosition((int)HIGH_BASKET)
                                ),
                                fourSampleAuto7,
                                new ParallelAction(
                                        wrist1.downWrist(),
                                        Intake1.depositIntake()

                                ),
                                new SleepAction(1),
                                new ParallelAction(
                                        fourSampleAuto8,
                                        arm.moveToPosition((int)ARM_CLEAR_BARRIER)
                                )

                        ));











            }



}

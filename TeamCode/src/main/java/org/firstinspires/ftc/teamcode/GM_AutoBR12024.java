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
@Autonomous(name = "GM_AutoBR12024",group="Autonomous")
public class GM_AutoBR12024 extends LinearOpMode {
    double kp=0.77
            ,ki=0.003
            ,kd=0.004;
    int GROUND_POS=0;
    int ARM_CLEAR_BARRIER=50;
    int LOW_BASKET=280;
    int HIGH_BASKET=320;
    double armPosition;
    PIDController pid1=new PIDController(kp,ki,kd);
    double armPower,armPower1;

    public class Arm {
        DcMotorEx armRotator=null,
                armRotator2=null;



        public Arm(HardwareMap hardwareMap) {
            armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
            armRotator.setDirection(DcMotor.Direction.FORWARD);
            armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
            armRotator2.setDirection(DcMotor.Direction.REVERSE);
            armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class MoveArmAction implements Action {
            int targetPosition;
            private boolean initialized = false;

            public MoveArmAction(int targetPosition) {
                this.targetPosition = targetPosition;
               // new SleepAction(1);

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    armPower = pid1.update(targetPosition,armRotator.getCurrentPosition(),15);
                    armPower1=pid1.update(targetPosition,armRotator.getCurrentPosition(),15);

                    armRotator.setTargetPosition(targetPosition);
                    armRotator.setPower(armPower);
                   // armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRotator2.setTargetPosition(targetPosition);
                    armRotator2.setPower(armPower1);
                   // armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                packet.put("Arm1 Position", armRotator.getCurrentPosition());
                packet.put("Arm2 Position", armRotator2.getCurrentPosition());
                return false;

            }
        }


        public Action moveArmUpperBasket() {

            return new MoveArmAction(HIGH_BASKET);
        }
        public Action moveArmLowerBasket(){
            return new MoveArmAction(LOW_BASKET);
        }
        public Action moveArmClearBarrier(){
            return new MoveArmAction(ARM_CLEAR_BARRIER);
        }
        public Action moveArmGroundPosition(){
            return new MoveArmAction(GROUND_POS);
        }
    }

    //Blue Right to Park in Observation Zone
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-24, 62, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Action onePlusThreeSpecimen1 = drive.actionBuilder(drive.pose)
                 .strafeToLinearHeading(new Vector2d(-40,62), Math.toRadians(270))
                .build();
        Actions.runBlocking(
                onePlusThreeSpecimen1
        );


    }
}

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
    public static String TEAM_NAME = "GreenMachine"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 8791;
    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUERIGHT,
        REDRIGHT
    }
    public static START_POSITION startPosition;

    //Blue Right to Park in Observation Zone
    @Override
    public void runOpMode() throws InterruptedException {
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Right   ", "(X / ▢)");
            telemetry.addData("    Red Right ", "(Y / Δ)");

            if(gamepad1.x){
                startPosition = START_POSITION.BLUERIGHT; //Blue Left
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.REDRIGHT; //Red
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
            Pose2d initialPose = new Pose2d(-24, 62, Math.toRadians(270));
            PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
            Action onePlusThreeSpecimen1 = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(-55,62), Math.toRadians(270))
                    .build();
            Actions.runBlocking(
                    onePlusThreeSpecimen1
            );


        }


    }// end runOpMode()



    }


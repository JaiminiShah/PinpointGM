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

    //Blue Right to Park in Observation Zone
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-24, 62, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Action onePlusThreeSpecimen1 = drive.actionBuilder(drive.pose)
                 .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(270))
                .build();
        Actions.runBlocking(
                onePlusThreeSpecimen1
        );


    }
}

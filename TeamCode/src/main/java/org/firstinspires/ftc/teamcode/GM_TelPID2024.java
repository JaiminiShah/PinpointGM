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

@Config

@TeleOp(name="GM_TelPID2024", group="GreenMachine")

public class GM_TelPID2024 extends OpMode {
    DcMotorEx armRotator = null,
            armRotator2 = null,
            armSlide = null;
    private final double zeroOffset = 90;

    private PIDFController pidf1;
    private PIDFController pidf2;
    public static  double kp=0.77;//0.77;
    public static  double ki=0.003;//0.003;
    public static  double kd=0.004;//0.004;

    public static double kf=0.03;//0.03;

    public static int targetMotorPosition=0;

    //public static int targetDeg=0;
    private final double ticksPerDegree=1425 / 360;
    public  Encoder armMotorEncoder;

    @Override
    public void init() {
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        pidf1 = new PIDFController(kp, ki,kd, kf);// Motor1 pidf
        pidf2=new PIDFController(kp,ki,kd,kf);// Motor2 pidf
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armRotator.setDirection(DcMotor.Direction.REVERSE);
        armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        // Adjust the target position based on gamepad input
        if ( gamepad1.dpad_up){
            targetMotorPosition+=5; // Increase target position
        } else if (gamepad1.dpad_down){
            targetMotorPosition-=5; // Decrease target position

        }
        // Get current motor positions
        double currentPosition1=armRotator.getCurrentPosition();
        double currentPosition2=armRotator2.getCurrentPosition();
        // Compute PIDF outputs
        double power1 = pidf1.calculate(currentPosition1,targetMotorPosition);
        double power2=pidf2.calculate(currentPosition2,targetMotorPosition);

        // Apply synchronized control
        armRotator.setPower(power1);
        armRotator2.setPower(power2);
        telemetry.addData("Target Position", targetMotorPosition);
        telemetry.addData("Motor1 Position", currentPosition1);
        telemetry.addData("Motor2 Position", currentPosition2);
        telemetry.addData("Motor1 Power", power1);
        telemetry.addData("Motor2 Power", power2);
        telemetry.update();


    }

    /*private void synchronizeMotors(int targetPosition){
        // Get the current positions of the motors
        int position1=armRotator.getCurrentPosition();
        int position2=armRotator2.getCurrentPosition();
        // Calculate the error for Motor1 (target-current)
        double error1=targetPosition-position1;
        // Calculate the error for Motor2 ( to sync with Motor1)
        double error2=position1-position2;
        // Calculate PIDF output for Motor1


    }*/
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="GM_Teleop12024", group="GreenMachine")

public class GM_Teleop12024 extends OpMode {
    DcMotorEx armRotator = null,
            armRotator2 = null;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    double armPower,armPower1;
    double kp=0.77
            ,ki=0.003
            ,kd=0.004;
    double GROUND_POS=0;
    double ARM_CLEAR_BARRIER=200;
    double LOW_BASKET=320;
    double HIGH_BASKET=350;
    double armPosition;
    final double LIFT_COLLAPSED = 0;
    final double liftIncrement             = 23.0;
    final double MAX_ARM_POS = 2000;


    DcMotorEx
            armSlide = null;

    PIDController pid1=new PIDController(kp,ki,kd);

    double liftPosition = LIFT_COLLAPSED;

    @Override
    public void init() {
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        //Direction of motors
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator2.setDirection(DcMotor.Direction.REVERSE);
        // Sets up motors
        armRotator.setPower(0);
        armRotator2.setPower(0);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        //telemetrylift();
        telemetry.update();
        runTime.reset();
        armSlide.setDirection(DcMotor.Direction.REVERSE);
        armSlide.setPower(0);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
        // Controls for armRotator
        if (gamepad2.a) {

            armPower = pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),8);
            armPosition=LOW_BASKET;
            // wrist.setPosition(.7);
        }

        else if (gamepad2.b) {
            armPower = pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),16);
            armPower1=pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),16);
            armPosition=HIGH_BASKET;
            // armPosition = LOW_BASKET;
            //wrist.setPosition(.5);
        }

        else if (gamepad2.y) {
            armPower = pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
            armPosition = ARM_CLEAR_BARRIER;
            //wrist.setPosition(.5);
        }

        armRotator.setTargetPosition((int)(armPosition));
        armRotator.setPower(armPower);
        //((DcMotorEx) armRotator).setVelocity(200);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator2.setTargetPosition((int) (armPosition));
        armRotator2.setPower(armPower1);
        // ((DcMotorEx) armRotator2).setVelocity(200);
        armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Controls for arm slide (real)
        if (gamepad1.left_trigger > .1 && liftPosition > 0) {
            liftPosition-=liftIncrement;
        }

        if (gamepad1.right_trigger > .1 && liftPosition<1970) {
            liftPosition+=liftIncrement;
        }

        // Makes sure the lift does not go beyond parameters
        if (liftPosition > MAX_ARM_POS){
            liftPosition = MAX_ARM_POS;
        }

        else if (liftPosition < 0){
            liftPosition = 0;
        }

        // Moves slide to the position
        armSlide.setTargetPosition((int) (liftPosition));

        ((DcMotorEx) armSlide).setVelocity(400);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (armRotator.isOverCurrent()) {
            telemetry.addLine("Rotator 1 Exceeded Limit");
        }

        if (armRotator2.isOverCurrent()) {
            telemetry.addLine("Rotator 2 Exceeded Limit");
        }

        if (armSlide.isOverCurrent()) {
            telemetry.addLine("Slide Exceeded Limit");
        }
    }

    public void telemetrymotorprint() {
        telemetry.clear();
        telemetry.addData("Arm Motor 1 Current", (armRotator.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("Arm Motor 2 Current", (armRotator2.getCurrent(CurrentUnit.AMPS)));

    }
}

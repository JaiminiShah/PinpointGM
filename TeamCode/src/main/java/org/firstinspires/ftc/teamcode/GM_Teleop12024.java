package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double ARM_CLEAR_BARRIER=25;
    double LOW_BASKET=700;
    double HIGH_BASKET=1000;
    double armPosition;
    PIDController pid1=new PIDController(kp,ki,kd);

    @Override
    public void init() {
        armRotator = hardwareMap.get(DcMotorEx.class, "armRotator");
        armRotator2 = hardwareMap.get(DcMotorEx.class, "armRotator2");
        //Direction of motors
        armRotator.setDirection(DcMotor.Direction.REVERSE);
        armRotator2.setDirection(DcMotor.Direction.FORWARD);
        // Sets up motors
        armRotator.setPower(0);
        armRotator2.setPower(0);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        //telemetrylift();
        telemetry.update();
        runTime.reset();

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
            armPower = pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),8);
            armPower1=pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),8);
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


    }

}

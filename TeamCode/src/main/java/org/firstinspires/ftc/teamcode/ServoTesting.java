package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class ServoTesting extends OpMode {
    ServoImplEx wrist;

    @Override
    public void init() {
        wrist = hardwareMap.get(ServoImplEx.class, "claw");
    }

    @Override
    public void loop() {
        if (gamepad1.a) wrist.setPosition(0);
        if (gamepad1.b) wrist.setPosition(0.25);
        if (gamepad1.left_bumper) wrist.getController().pwmDisable();
        if (gamepad1.left_trigger > 0.75) wrist.getController().pwmEnable();

        telemetry.addData("claw cont", wrist.getController());
        telemetry.addData("Claw pwm", wrist.isPwmEnabled());
        telemetry.update();
    }
}

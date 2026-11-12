package org.firstinspires.ftc.teamcode.tele26;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Drive motors

@TeleOp(name = "shooter trial", group = "TeleOp")
public class shooterTest extends OpMode {

    DcMotorEx shooter;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize your hardware here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0) {
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }

        // Main loop logic here
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}

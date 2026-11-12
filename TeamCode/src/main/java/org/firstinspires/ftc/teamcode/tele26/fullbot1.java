package org.firstinspires.ftc.teamcode.tele26;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Drive motors

@Configurable

@TeleOp(name = "Full Bot 1", group = "TeleOp")
public class fullbot1 extends OpMode {

    DcMotorEx lf, lr, rf, rr;
    DcMotorEx turret;
    DcMotorEx shooter;
    DcMotorEx intake1, intake2;

    // Power multiplier for drive
    double powerMult = 0.9;

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize your hardware here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Main loop logic here
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}

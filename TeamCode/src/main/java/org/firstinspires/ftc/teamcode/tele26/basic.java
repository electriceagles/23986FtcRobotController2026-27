package org.firstinspires.ftc.teamcode.tele26;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "basic drive", group = "TeleOp")
public class basic extends OpMode {

    // Drive motors
    DcMotorEx lf, lr, rf, rr;

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
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        lf.setPower((y + x + rx * 0.8) * powerMult);
        rf.setPower((y - x - rx * 0.8) * powerMult);
        lr.setPower((y - x + rx * 0.8) * powerMult);
        rr.setPower((y + x - rx * 0.8) * powerMult);
    }
}
package org.firstinspires.ftc.teamcode.tele26;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Drive motors

@Configurable

@TeleOp(name = "Full Bot 1", group = "TeleOp")
public class fullbot1 extends OpMode {

    DcMotorEx lf, lr, rf, rr, intake;
    DcMotorEx turret;
    DcMotorEx shooter;
    DcMotorEx intake1, intake2;


    // Shooter
    public static double curTargetVelocity = -1500;


    public static double P = 120.0400;
    public static double F = 14.9600;

    // Odometry
    GoBildaPinpointDriver pinpoint;

    // Power multiplier for drive
    double powerMult = 0.9;

    @Override
    public void init() {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);


        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2"); //if go right up

        // Turret
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);

        intake1.setDirection(DcMotorSimple.Direction.FORWARD); //goes right in
        intake2.setDirection(DcMotorSimple.Direction.FORWARD); // goes right up


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-6.518500620924582, -8.005260557640257, DistanceUnit.INCH);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));

        telemetry.addLine("Starting Pose: x, y, theta");


        // Initialize your hardware here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {


        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        lf.setPower((y - x + rx * 0.8) * powerMult);
        rf.setPower((y + x - rx * 0.8) * powerMult);
        lr.setPower((y + x + rx * 0.8) * powerMult);
        rr.setPower((y - x - rx * 0.8) * powerMult);

        if (gamepad1.left_bumper) {
            intake1.setPower(1);
            intake2.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        }
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        if (gamepad1.right_trigger > 0.1) {
            shooter.setVelocity(curTargetVelocity);
        } else {
            shooter.setVelocity(0);
        }

        double curVelocity1 = shooter.getVelocity();
        double error1 = curVelocity1 - curTargetVelocity;


        // Main loop logic here
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}

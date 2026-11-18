package org.firstinspires.ftc.teamcode.tele26;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// Drive motors

@Configurable
@TeleOp(name = "shooter trial", group = "TeleOp")
public class shooterTest extends OpMode {

    DcMotorEx shooter;

    public static double highV = 1500; // tune to minimize error for this, and then figure out optimal velocity for hives
    public double lowV = 900;

    public static double curTargVel = highV;
    public double P = 0;
    public double F = 0;
    double[] stepSize = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;



    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        // Initialize your hardware here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed()) {
            if (curTargVel == highV) {
                curTargVel = lowV;
            } else {
                curTargVel = highV;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSize.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSize[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()) {
            F+=stepSize[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSize[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()) {
            P-=stepSize[stepIndex];
        }

        // new pidf values

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        shooter.setVelocity(curTargVel);

        double curVel = shooter.getVelocity();
        double error = curTargVel-curVel;

        telemetry.addData("target velocity", curTargVel);
        telemetry.addData("current velocity", "%.2f", curVel);
        telemetry.addData("error", "%.2f", error);
        telemetry.addLine("_____________________________________________________");

        telemetry.addData("P:", "%.4f", P);
        telemetry.addData("F:", "%.4f", F);
        telemetry.addData("step size (click B):", "%.4f", stepSize[stepIndex]);




    }
}

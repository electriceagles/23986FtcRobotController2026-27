package org.firstinspires.ftc.teamcode.auton.red;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

@Autonomous
public class oneTipRed extends OpMode {
    private DcMotor intake1, intake2;
    private DcMotorEx shooter;

    public static double curTargetVelocity = -1500;
    public static double P = 120.0400;
    public static double F = 14.9600;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose score = poseFactory.of(56, 24, 90);
    private final Pose park = poseFactory.of(10.5802, 91.4761, 107.3855);
    private final Pose parkControl1 = poseFactory.of(19.6097, 62.2452, 0);

    public Path score() {
        return line(startPose, score).linear(startPose, score);
    }

    public Path park() {
        return curve(score, parkControl1, park).tangent();
    }
    private Command shoot() {
        // Wait commands must be scheduled as steps, not called inside setStart.
        return sequential(
                hold(follower, score),
                instant(() -> shooter.setPower(curTargetVelocity)),
                waitMs(500),
                instant(() -> {
                    intake1.setPower(1);
                    intake2.setPower(1);
                }),
                waitMs(5000),
                instant(this::stopShooting)
        );
    }

    private void stopShooting() {
        intake1.setPower(0);
        intake2.setPower(0);
        shooter.setPower(0);
    }

    private Command autoRoutine(){
        return sequential(
                follow(follower, score()),
                shoot(),
                follow(follower, park())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        stopShooting();
    }
}

package org.firstinspires.ftc.teamcode.auton.blue;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.*;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

@Autonomous
public class Blue2Tip extends OpMode {
    private DcMotor intake1, intake2;
    private DcMotorEx shooter;
    private Servo flower;

    public static double curTargetVelocity = -1500;
    public static double P = 120.0400;
    public static double F = 14.9600;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84.0529, 130.9288, 90);
    private final Pose start2shoot = poseFactory.of(84, 104.8049, 270);
    private final Pose toFlower = poseFactory.of(130, 95, 0);
    private final Pose hive2Start = poseFactory.of(130, 95, 0);
    private final Pose hive2 = poseFactory.of(84, 22, 90);
    private final Pose hive2Control1 = poseFactory.of(115.5784, 40.8557, 0);
    private final Pose flower2 = poseFactory.of(94.6278, 11.525, -89.7104);
    private final Pose flower2Control1 = poseFactory.of(84.6695, 38.7978, 0);
    private final Pose flower2Control2 = poseFactory.of(94.5003, 36.7603, 0);
    private final Pose tip2Start = poseFactory.of(94.6278, 11.525, 270);
    private final Pose tip2 = poseFactory.of(85, 22.3558, 90);
    private final Pose tip2Control1 = poseFactory.of(57.1682, 23.1972, 0);
    private final Pose tip2Control2 = poseFactory.of(84.6472, 46.9659, 0);
    private final Pose toParkStart = poseFactory.of(85, 22.3558, 0);
    private final Pose toPark = poseFactory.of(126.1764, 27.953, 0);

    public Path start2shoot() {
        return Paths.line(start, start2shoot).constant(start2shoot);
    }
    public Path toFlower() {
        return Paths.line(start2shoot, toFlower).constant(toFlower);
    }
    public Path hive2() {
        return Paths.curve(hive2Start, hive2Control1, hive2).linear(hive2Start, hive2);
    }
    public Path flower2() {
        return Paths.curve(hive2, flower2Control1, flower2Control2, flower2).tangent();
    }
    public Path tip2() {
        return Paths.curve(tip2Start, tip2Control1, tip2Control2, tip2).linear(tip2Start, tip2);
    }
    public Path toPark() {
        return Paths.line(toParkStart, toPark).linear(toParkStart, toPark);
    }

    private Command shoot() {
        // Wait commands must be scheduled as steps, not called inside setStart.
        return sequential(
                hold(follower),
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

    private Command flowerDown() {
        return sequential(
                hold(follower),
                instant(() -> flower.setPosition(0.5))
        );
    }
    private Command flowerUp() {
        return sequential(
                hold(follower),
                instant(() -> flower.setPosition(0))
        );
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, start2shoot()),
                shoot(),
                flowerDown(),
                follow(follower, toFlower()),
                follow(follower, hive2()),
                flowerUp(),
                shoot(),
                flowerDown(),
                follow(follower, flower2()),
                follow(follower, tip2()),
                flowerUp(),
                shoot(),
                follow(follower, toPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start2shoot);
        follower.update();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        flower = hardwareMap.get(Servo.class, "flower");

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
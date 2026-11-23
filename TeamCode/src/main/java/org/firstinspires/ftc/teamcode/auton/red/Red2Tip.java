package org.firstinspires.ftc.teamcode.auton.red;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
public class Red2Tip extends OpMode {
    private DcMotor intake1, intake2;
    private DcMotorEx shooter;
    private Servo flower;

    public static double curTargetVelocity = -1500;
    public static double P = 120.0400;
    public static double F = 14.9600;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.4471, 10.5712, 90);
    private final Pose start2shoot = poseFactory.of(57.5, 36.6951, 90);
    private final Pose toFlower = poseFactory.of(11.1, 47, 180);
    private final Pose toFlowerControl1 = poseFactory.of(30, 31, 0);
    private final Pose toFlowerControl2 = poseFactory.of(23, 47, 0);
    private final Pose hive2Start = toFlower;
    private final Pose hive2 = poseFactory.of(57.5, 119.5, 270);
    private final Pose hive2Control1 = poseFactory.of(15, 47, 0);
    private final Pose hive2Control2 = poseFactory.of(30, 112, 0);
    private final Pose flower2 = poseFactory.of(46.75, 130.9, 90);
    private final Pose flower2Control = poseFactory.of(46.75, 120, 0);
    private final Pose tip2Start = flower2;
    private final Pose tip2 = poseFactory.of(56.5, 119.1442, 270);
    private final Pose tip2Control = poseFactory.of(46.75, 122, 0);
    private final Pose toParkStart = tip2;
    private final Pose toPark = poseFactory.of(15.3236, 113.547, 180);

    public Path start2shoot() {
        return Paths.line(start, start2shoot).constant(start2shoot);
    }

    public Path toFlower() {
        return Paths.curve(start2shoot, toFlowerControl1, toFlowerControl2, toFlower).linear(start2shoot, toFlower);
    }

    public Path hive2() {
        return Paths.curve(hive2Start, hive2Control1, hive2Control2, hive2).linear(hive2Start, hive2);
    }

    public Path flower2() {
        return Paths.curve(hive2, flower2Control, flower2).linear(hive2, flower2);
    }

    public Path tip2() {
        return Paths.curve(tip2Start, tip2Control, tip2).linear(tip2Start, tip2);
    }

    public Path toPark() {
        return Paths.line(toParkStart, toPark).linear(toParkStart, toPark);
    }

    private Command shoot() {
        return sequential(
                hold(follower)
                        .setStart(() -> shooter.setVelocity(curTargetVelocity))
                        .setExecute(() -> {
                            waitMs(1500);
                            startIntake();
                            waitMs(2500);
                        })
                        .setDone(() -> true)
                        .setEnd(endCondition -> stopShooting())
        );
    }
    private void startIntake() {
        intake1.setPower(1);
        intake2.setPower(1);
    }

    private void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    private void stopShooting() {
        stopIntake();
        shooter.setVelocity(0);
    }

    private Command flowerDown() {
        return sequential(
                waitMs(500),
                instant(() -> flower.setPosition(0.5))
        );
    }
    private Command flowerUp() {
        return sequential(
                waitMs(500),
                instant(() -> flower.setPosition(0))
        );
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, start2shoot()),
                shoot(),
                flowerDown().with(follow(follower, toFlower())),
                follow(follower, hive2()).with(flowerUp()),
                shoot(),
                flowerDown().with(follow(follower, flower2())),
                follow(follower, tip2()).with(flowerUp()),
                shoot(),
                follow(follower, toPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        flower = hardwareMap.get(Servo.class, "flower");
        flower.setPosition(0);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);

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

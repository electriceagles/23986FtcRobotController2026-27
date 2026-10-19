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
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class oneTipRed extends OpMode {
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
    private Command autoRoutine(){
        return sequential(
                follow(follower, score()),
                //ToDo: add the motor control routine (intake + outtake)
                follow(follower, park())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
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
}
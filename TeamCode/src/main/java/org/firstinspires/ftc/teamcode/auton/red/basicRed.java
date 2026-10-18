package org.firstinspires.ftc.teamcode.auton.red;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous
public class basicRed extends OpMode {

    private Follower follower;

    /*Note: We are using degrees because I find it easier.
    In the future if you want to change it, switch .degrees() to .radians() */
    private final PoseFactory p = PoseFactory.degrees();

    /* Poses specifying the startpoint and endpoint of the path we want the robot to follow.
    Use https://pedropathing.com/docs/pathing/reference/coordinates for coordinate reference.*/
    private final Pose startPose = p.of(6, 138, 270);
    private final Pose park = p.of(10, 120, 270);
    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }
    @Override
    public void start() {
        schedule(follow(follower, park()));
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
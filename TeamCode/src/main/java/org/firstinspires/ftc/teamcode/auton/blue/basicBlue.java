package org.firstinspires.ftc.teamcode.auton.blue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
@Autonomous
public class basicBlue extends OpMode {

    private Follower follower;

    /*Note: We are using degrees because I find it easier.
    In the future if you want to change it, switch .degrees() to .radians() */
    private final PoseFactory p = PoseFactory.degrees();

    /* Poses specifying the startpoint and endpoint of the path we want the robot to follow.
    Use https://pedropathing.com/docs/pathing/reference/coordinates for coordinate reference.*/
    private final Pose startPose = p.of(0, 0, 0);
    private final Pose park = p.of(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
    }
}
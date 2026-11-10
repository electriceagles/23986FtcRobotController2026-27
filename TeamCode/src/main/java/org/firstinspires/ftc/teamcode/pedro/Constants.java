package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static PinpointConfig localizerConfig;
    public static MecanumConfig drivetrainConfig;

    public static Follower create(HardwareMap h) {
        return null;
        //return new Follower(Drivetrain, Localizer, Foresight);
    }
}
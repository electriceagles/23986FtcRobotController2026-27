package org.firstinspires.ftc.teamcode.tea;

import static org.firstinspires.ftc.teamcode.tea.Globals.F;
import static org.firstinspires.ftc.teamcode.tea.Globals.P;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class RobotHardware {

    // Add variables here and keep them public for now
    public DcMotorEx lf, lr, rf, rr;
    public DcMotorEx turret;
    public DcMotorEx shooter;
    public DcMotorEx intake1, intake2;

    public GoBildaPinpointDriver pinpoint;

    // The initialization function that you will call in your init() loop of every autonomous or teleop
    /**
     *  RobotHardware robotHardware = new RobotHardware();
     *
     * int() {
     *     robotHardware.initHardware(hardwareMap, telemetry);
     * }
     *
     * The above snippet is an example of how you can initialize and utilize the function
     * To initialize your robots hardware that's all you'll have to do in any file.
     *
     * @param hardwareMap - where your "HardwareMap" variable will be assigned
     * @param telemetry - your telemetry goes here if you have any being called in a function for now status initialized is fine
     */
    public void initHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Turret: encoder reset here, so the turret MUST be pointing straight forward at INIT
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F)); // P and F are static imports from the file "Global"

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setOffsets(-6.518500620924582, -8.005260557640257, DistanceUnit.INCH);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 93, 0, AngleUnit.DEGREES, 90));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}

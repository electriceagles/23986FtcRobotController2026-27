package org.firstinspires.ftc.teamcode.tele26;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//sus
@Configurable
@TeleOp(name = "Full Bot Odo Tur", group = "TeleOp")
public class fullBot2 extends OpMode {

    DcMotorEx lf, lr, rf, rr;
    DcMotorEx turret;
    DcMotorEx shooter;
    DcMotorEx intake1, intake2;

    public static double curTargetVelocity = -1500;
    public static double P = 120.0400;
    public static double F = 14.9600;

    GoBildaPinpointDriver pinpoint;

    double powerMult = 0.9;

    // TODO: hive pos in the same frame as the Pinpoint pose (inches)
    public static double HIVE_X = 12.75;
    public static double HIVE_Y = 17;

    // Belt: 32T motor gear , tur= 140T turret gear
    static final double GEAR_RATIO = 140.0 / 32.0;
    // TODO: ticks per  rev for turret motor
    public static double MOTOR_TICKS_PER_REV = 1425.1;

    // Turret angle (deg) when the encoder is reset in init(). 0 = pointing straight forward.
    public static double TURRET_START_DEG = 0.0;

    // Travel limits in degrees (keep cables from wrapping)
    public static double TURRET_MIN_DEG = -180;
    public static double TURRET_MAX_DEG = 90;

    // PID (tunable  from Panels)
    public static double kP = 1.6;
    public static double kI = 0.0;
    public static double kD = 0.12;
    public static double kF = 0.0;
    public static double MAX_TURRET_POWER = 0.8;
    public static double DEADBAND_DEG = 0.5;
    public static double INTEGRAL_LIMIT = 0.3;

    private boolean aiming = false;
    private boolean lastTriangle = false;
    private double integral = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

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
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Turret: encoder reset here, so start every run with the turret pointing forward
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setOffsets(-6.518500620924582, -8.005260557640257, DistanceUnit.INCH);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 24, -67.71, AngleUnit.DEGREES, 90));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        pidTimer.reset();
    }

    @Override
    public void loop() {
        // must be called every loop or the pose never updates
        pinpoint.update();

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        lf.setPower((y + x - rx * 0.8) * powerMult);
        rf.setPower((y - x + rx * 0.8) * powerMult);
        lr.setPower((y - x - rx * 0.8) * powerMult);
        rr.setPower((y + x - rx * 0.8) * powerMult);

        if (gamepad1.left_bumper) {
            intake1.setPower(1);
            intake2.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        } else {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        if (gamepad1.right_trigger > 0.1) {
            shooter.setVelocity(curTargetVelocity);
        } else {
            shooter.setVelocity(0);
        }

        boolean triangle = gamepad1.triangle;
        if (triangle && !lastTriangle) {
            aiming = !aiming;
            integral = 0;
            lastError = 0;
            pidTimer.reset();
        }
        lastTriangle = triangle;

        Pose2D pose = pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        // Turret is at robot center, so robot position = turret position
        double dx = HIVE_X - robotX;
        double dy = HIVE_Y - robotY;
        double fieldAngle = Math.atan2(dy, dx);

        // Angle relative to robot heading is  desired turret angle
        double desired = applyLimits(wrap(fieldAngle - heading));

        double ticksPerRad = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);
        double current = Math.toRadians(TURRET_START_DEG) + turret.getCurrentPosition() / ticksPerRad;
        double error = desired - current;

        double power = 0;
        if (aiming) {
            power = pid(error);
        }
        turret.setPower(power);

        telemetry.addData("Aiming", aiming);
        telemetry.addData("Robot x,y,hdg", "%.1f, %.1f, %.1f", robotX, robotY, Math.toDegrees(heading));
        telemetry.addData("Desired turret deg", "%.1f", Math.toDegrees(desired));
        telemetry.addData("Current turret deg", "%.1f", Math.toDegrees(current));
        telemetry.addData("Error deg", "%.2f", Math.toDegrees(error));
        telemetry.addData("Turret power", "%.2f", power);
        telemetry.update();
    }

    private double pid(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0 || dt > 0.25) dt = 0.02;

        if (Math.abs(error) < Math.toRadians(DEADBAND_DEG)) {
            integral = 0;
            lastError = error;
            return 0;
        }

        integral = Range.clip(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double out = kP * error + kI * integral + kD * derivative + kF * Math.signum(error);
        return Range.clip(out, -MAX_TURRET_POWER, MAX_TURRET_POWER);
    }

    // ---------------- Angle helpers ----------------
    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /** Pick the equivalent angle inside the turret's travel range, else clamp. */
    private static double applyLimits(double a) {
        double min = Math.toRadians(TURRET_MIN_DEG);
        double max = Math.toRadians(TURRET_MAX_DEG);
        if (a >= min && a <= max) return a;
        double up = a + 2 * Math.PI;
        double down = a - 2 * Math.PI;
        if (up >= min && up <= max) return up;
        if (down >= min && down <= max) return down;
        return Range.clip(a, min, max);
    }
}
package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** Camera must be rigidly mounted to the rotating shooter. See LIMELIGHT_AUTO.md. */
public abstract class LimelightAuto extends OpMode {
    // Configure before enabling. Negative target IDs intentionally prevent shooting.
    public static int RED_TARGET_ID = -1, BLUE_TARGET_ID = -1;
    public static int RED_PIPELINE = 0, BLUE_PIPELINE = 1;
    public static double AIM_KP = 0.02, ROTATE_SIGN = 1, MAX_ROTATE_POWER = 0.25;
    public static double AIM_OFFSET_DEG = 0, TOLERANCE_DEG = 1.5;
    // Relative to the manually centered turret at INIT; replace with measured safe limits.
    public static int MIN_ROTATE_TICKS = 0, MAX_ROTATE_TICKS = 0;
    private Follower follower;
    private DcMotor rotate, shooter, intake1, intake2;
    private Limelight3A limelight;
    private final ElapsedTime phase = new ElapsedTime(), stable = new ElapsedTime();
    private int origin;
    private boolean aligned;
    private double error = Double.NaN;
    protected abstract boolean red();

    @Override public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        origin = rotate.getCurrentPosition();
        stopMotors();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(red() ? RED_PIPELINE : BLUE_PIPELINE);
        limelight.start();
        telemetry.addLine("Configure goal IDs and turret limits; center turret before INIT.");
    }
    private Pose pose(double x, double y, double h) { return PoseFactory.degrees().of(x,y,h); }
    private Pose startPose() { return red() ? pose(56,8,90) : pose(85,134,270); }
    private Pose scorePose() { return red() ? pose(56,24,90) : pose(85,120,270); }
    private Pose parkPose() { return red() ? pose(10.5802,91.4761,107.3855) : pose(132,48,-75.8006); }
    private Pose controlPose() { return red() ? pose(19.6097,62.2452,0) : pose(120,96,0); }

    private Command shoot() {
        return Command.build()
            .setStart(() -> { phase.reset(); stable.reset(); aligned = false; shooter.setPower(1); })
            .setExecute(() -> {
                updateAim();
                boolean feed = aligned && stable.seconds() >= 0.25 && phase.seconds() >= 0.5;
                intake1.setPower(feed ? 1 : 0);
                intake2.setPower(feed ? 1 : 0);
            })
            .setDone(() -> phase.seconds() >= 6)
            .setEnd(reason -> stopMotors())
            .requiring(rotate, shooter, intake1, intake2);
    }
    private void updateAim() {
        error = Double.NaN;
        LLResult result = limelight.getLatestResult();
        int target = red() ? RED_TARGET_ID : BLUE_TARGET_ID;
        if (target >= 0 && result != null && result.isValid() && result.getStaleness() <= 150
                && result.getPipelineIndex() == (red() ? RED_PIPELINE : BLUE_PIPELINE)) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == target) {
                    error = tag.getTargetXDegrees() - AIM_OFFSET_DEG;
                    break;
                }
            }
        }
        aligned = Double.isFinite(error) && Math.abs(error) <= TOLERANCE_DEG;
        if (!aligned) stable.reset();
        double power = Double.isFinite(error) && !aligned
                ? Range.clip(ROTATE_SIGN * AIM_KP * error, -MAX_ROTATE_POWER, MAX_ROTATE_POWER) : 0;
        int ticks = rotate.getCurrentPosition() - origin;
        if (MIN_ROTATE_TICKS >= MAX_ROTATE_TICKS || (ticks <= MIN_ROTATE_TICKS && power < 0)
                || (ticks >= MAX_ROTATE_TICKS && power > 0)) power = 0;
        rotate.setPower(power);
    }
    @Override public void start() {
        Scheduler.schedule(sequential(
            follow(follower, line(startPose(), scorePose()).linear(startPose(), scorePose())),
            shoot(),
            follow(follower, curve(scorePose(), controlPose(), parkPose()).tangent())));
    }
    @Override public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("Goal ID", red() ? RED_TARGET_ID : BLUE_TARGET_ID);
        telemetry.addData("Aim error (degrees)", error);
        telemetry.addData("Aligned", aligned);
        telemetry.addData("Turret ticks from INIT", rotate.getCurrentPosition() - origin);
        telemetry.update();
    }
    private void stopMotors() {
        rotate.setPower(0); shooter.setPower(0); intake1.setPower(0); intake2.setPower(0);
    }
    @Override public void stop() {
        Scheduler.reset();
        stopMotors();
        if (limelight != null) limelight.stop();
    }
}

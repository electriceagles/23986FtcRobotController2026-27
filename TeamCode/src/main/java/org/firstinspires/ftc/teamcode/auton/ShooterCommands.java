package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class ShooterCommands {
    private static final double SHOT_SECONDS = 5.0;
    private static final double SPIN_UP_TIMEOUT_SECONDS = 3.0;
    private static final double VELOCITY_TOLERANCE = 0.05;

    private ShooterCommands() {
    }

    public static Command shoot(DcMotorEx shooter, DcMotor intake1, DcMotor intake2,
                                double targetVelocity) {
        class ShotTiming {
            final ElapsedTime timer = new ElapsedTime();
            boolean atSpeedOnce;
        }
        ShotTiming timing = new ShotTiming();
        double tolerance = Math.abs(targetVelocity) * VELOCITY_TOLERANCE;

        return Command.build()
                .setStart(() -> {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setVelocity(targetVelocity);
                    timing.atSpeedOnce = false;
                    timing.timer.reset();
                })
                .setExecute(() -> {
                    boolean ready = targetVelocity != 0
                            && Math.abs(shooter.getVelocity() - targetVelocity) <= tolerance;
                    if (!timing.atSpeedOnce && ready
                            && timing.timer.seconds() < SPIN_UP_TIMEOUT_SECONDS) {
                        timing.atSpeedOnce = true;
                        timing.timer.reset();
                    }

                    // Pause feeding during recovery without restarting the shot clock.
                    double power = timing.atSpeedOnce && ready
                            && timing.timer.seconds() < SHOT_SECONDS ? 1 : 0;
                    intake1.setPower(power);
                    intake2.setPower(power);
                })
                .setDone(() -> timing.timer.seconds() >=
                        (timing.atSpeedOnce ? SHOT_SECONDS : SPIN_UP_TIMEOUT_SECONDS))
                .setEnd(endCondition -> {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setVelocity(0);
                });
    }
}

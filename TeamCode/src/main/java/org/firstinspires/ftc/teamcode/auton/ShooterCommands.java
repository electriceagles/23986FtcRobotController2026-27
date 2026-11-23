package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class ShooterCommands {
    private static final double SHOT_SECONDS = 4.0;
    private static final double VELOCITY_TOLERANCE = 0.05;

    private ShooterCommands() {
    }

    public static Command shoot(DcMotorEx shooter, DcMotor intake1, DcMotor intake2,
                                double targetVelocity) {
        ElapsedTime shotTimer = new ElapsedTime();
        double tolerance = Math.abs(targetVelocity) * VELOCITY_TOLERANCE;

        return Command.build()
                .setStart(() -> {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setVelocity(targetVelocity);
                    shotTimer.reset();
                })
                .setExecute(() -> {
                    // Pause feeding while the flywheel is recovering between shots.
                    boolean ready = shotTimer.seconds() < SHOT_SECONDS
                            && targetVelocity != 0
                            && Math.abs(shooter.getVelocity() - targetVelocity) <= tolerance;
                    double power = ready ? 1 : 0;
                    intake1.setPower(power);
                    intake2.setPower(power);
                })
                .setDone(() -> shotTimer.seconds() >= SHOT_SECONDS)
                .setEnd(endCondition -> {
                    intake1.setPower(0);
                    intake2.setPower(0);
                    shooter.setVelocity(0);
                });
    }
}

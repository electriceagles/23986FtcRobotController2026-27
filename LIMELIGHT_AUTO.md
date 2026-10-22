# Limelight autonomous setup

Select `Limelight Red - Aim and Park` or `Limelight Blue - Aim and Park`.
These reuse the respective oneTip paths: drive to score, aim and shoot for a
maximum of six seconds, then park. Verify these existing coordinates on your field.

Configure hardware names: rotate, shooter, intake1, intake2, limelight.
The Limelight must rotate with the shooter; a chassis-fixed camera needs a different
geometry-based controller. Configure AprilTag pipelines (defaults red 0, blue 1).
Set RED_TARGET_ID and BLUE_TARGET_ID to the actual goal tag IDs in LimelightAuto.
They default to -1 to prevent accidental feeding at an unconfigured target.

Center the turret before INIT. Set MIN_ROTATE_TICKS and MAX_ROTATE_TICKS to measured
safe encoder offsets from this position. Both default to zero, disabling rotation.
Encoder soft limits require a working encoder and correct motor direction; these
are not a substitute for homing or physical limit switches. Verify ROTATE_SIGN at
low power so a positive error decreases, then tune AIM_KP, power, tolerance and
aim offset. Confirm shooter/intake directions and spin-up time on the robot.

A fresh matching goal must remain within tolerance for 250 ms before feeding,
and the shooter gets at least 500 ms spin-up. Feeding stops whenever alignment or
vision is lost. Target loss stops rotation; no blind target search is performed.
The six-second window includes acquisition and spin-up, so lost vision may result
in no shots. A timed spin-up does not measure shooter velocity. All mechanism
motors stop when the command ends or the OpMode stops. Existing autos are unchanged.

Uses Pedro 3.0.1 and Ivy 1.1.1 already declared by this repository.
Command builder reference: https://pedropathing.com/docs/ivy/command-builder

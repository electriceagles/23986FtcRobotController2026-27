# Two-tip and three-tip autonomous routines

Four new OpModes: Limelight Red / Blue - 2 Hive Tips and - 3 Hive Tips.
They use Pedro 3.0.1 and Ivy 1.1.1 already in this repository. No old routines are replaced.
The routines are deliberately @Disabled until measured routes and configuration exist.
Do not confuse two/three attempts with guaranteed scored tips.

Sequence: drive to shooting pose, aim and launch, infer a stable hive state change,
stop launching, wait for balls to settle, follow collection waypoints with both
intakes on, drive back to a shooting pose, and repeat until the requested number
of inferred tips. Then follow the corresponding parking route. No timer alone
counts a tip. A shot timeout, path timeout, missing configuration or overall
deadline stops the robot and mechanisms. It does not invent a parking path from
an arbitrary failed location. The 29-second deadline leaves the final stop margin;
you must measure whether your full three-tip route fits, including park time.

## Required robot setup

1. Implement Constants.create(hardwareMap): it currently returns null. Configure
   and tune the actual drivetrain and localizer; do not use guessed tuning.
   Existing Tuning.java also references missing localizerConfig and drivetrainConfig.
2. Fill TeamCode/src/main/assets/multitip-red.json and multitip-blue.json.
   Poses are [x inches,y inches,heading degrees]. Start poses were copied from
   oneTip routines; verify them too. Every route is consecutive straight line
   segments with linear heading interpolation. Include obstacle-clear intermediate
   waypoints; these are driven through, not Bezier control points.
3. Each cycle has collectRoute (empty only for the preload/first cycle) and
   shootRoute. Later collection routes begin at the previous shooting endpoint;
   the shooting route begins at the collection endpoint. Fill parkAfter2 and
   parkAfter3 starting from their respective last shooting endpoints.
4. Set two disjoint stateTagIds groups to tags that uniquely identify the two
   settled hive orientations from your actual shooting viewpoint. initialState
   is 0 or 1. The program expects alternation on each tip. Set pipeline to an
   AprilTag pipeline that can see BOTH groups, not a single-ID-only pipeline.
   A new orientation requires at least three distinct fresh frames for at least
   tipConfirmSeconds; missing/ambiguous frames reset confirmation. Before counting
   any transition, the expected orientation must be confirmed and feeding started.
   This is only a vision proxy for a settled tip, NOT the official damper-contact
   scoring criterion. Verify that the clusters genuinely distinguish stable states,
   and do not both appear at rest or during rotation. An unobservable transition
   causes a timeout, not a guessed tip. Do not enable if this sensing is unreliable.
5. The Limelight must rotate rigidly with the shooter. Tag offsets are calibrated
   crosshair offsets to launch into the upward-facing cell, not merely center the
   camera on a tag on a downward-facing cell. Every tag in a state group must give
   the same useful aiming reference; otherwise use one verified tag per group or
   implement per-tag geometry. Change aimOffsetDegrees for each stable state.
6. Hardware names: rotate, shooter, intake1, intake2, limelight. Manually center the
   turret BEFORE INIT. Set safe minRotateTicks/maxRotateTicks relative to that
   position, verify the working encoder and motor direction, tune rotateSign and
   aimKp at low power. Check intake/shooter directions and spin-up time. The code
   uses fixed shooter power; shooter speed/hood control must be tuned separately.
7. Measure tipSettleSeconds against actual ball drop/roll time and shotTimeoutSeconds
   against actual fill time. Feeding requires fresh target alignment for
   alignmentSeconds and spinupSeconds; target loss stops feeding and turret power.
8. Set configured=true, remove @Disabled from the desired wrappers, then test one
   segment and one tip before commissioning the full two/three-tip runs.

The JSON route arrays are empty on purpose: no collection measurements were
provided. Their initial settings are starting hypotheses, not calibrated values.
The standalone hive-prediction workspace is separate and never commands hardware.

Rules reference (tip definition, launching, hive structure):
https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html
Ivy command builder: https://pedropathing.com/docs/ivy/command-builder

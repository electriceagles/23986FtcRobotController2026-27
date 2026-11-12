package org.firstinspires.ftc.teamcode.auton.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.auton.MultiTipAuto;

@Disabled // Remove only after MULTITIP_SETUP.md commissioning steps.
@Autonomous(name = "Limelight Red - 3 Hive Tips")
public class Limelight3TipRed extends MultiTipAuto {
    @Override protected boolean red() { return true; }
    @Override protected int requestedTips() { return 3; }
}

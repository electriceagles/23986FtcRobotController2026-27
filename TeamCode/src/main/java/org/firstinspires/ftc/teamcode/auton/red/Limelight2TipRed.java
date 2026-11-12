package org.firstinspires.ftc.teamcode.auton.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.auton.MultiTipAuto;

@Disabled // Remove only after MULTITIP_SETUP.md commissioning steps.
@Autonomous(name = "Limelight Red - 2 Hive Tips")
public class Limelight2TipRed extends MultiTipAuto {
    @Override protected boolean red() { return true; }
    @Override protected int requestedTips() { return 2; }
}

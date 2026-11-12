package org.firstinspires.ftc.teamcode.auton.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.auton.MultiTipAuto;

@Disabled // Remove only after MULTITIP_SETUP.md commissioning steps.
@Autonomous(name = "Limelight Blue - 2 Hive Tips")
public class Limelight2TipBlue extends MultiTipAuto {
    @Override protected boolean red() { return false; }
    @Override protected int requestedTips() { return 2; }
}

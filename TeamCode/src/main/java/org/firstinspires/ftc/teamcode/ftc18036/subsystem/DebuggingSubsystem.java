package org.firstinspires.ftc.teamcode.ftc18036.subsystem;

import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.framework.Subsystem;

public class DebuggingSubsystem extends Subsystem {


    public DebuggingSubsystem(String name, BotManager botmgr, double rate) {
        super(name, botmgr, rate);
    }

    @Override
    public void execute() {
        opmode.telemetry.addData("IMU: ", botManager.getMeasurementPackage().orientationInRadians);
        opmode.telemetry.addData("Controller1 left y ", botManager.getMeasurementPackage().gamepad1.leftStickY);
        opmode.telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.botcore.subsystems;

import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.framework.Subsystem;
import org.firstinspires.ftc.teamcode.botcore.subsystems.sensing.ImuSensing;

public class ImuSensingSubsystem extends Subsystem {

    ImuSensing imuSensing;
    public ImuSensingSubsystem(String name, BotManager botmgr, double rate) {
        super(name, botmgr, rate);

        imuSensing = new ImuSensing(botmgr, botmgr.getConfiguration().sensing.kImuInterval);
    }

    @Override
    public void execute() {

    }
}

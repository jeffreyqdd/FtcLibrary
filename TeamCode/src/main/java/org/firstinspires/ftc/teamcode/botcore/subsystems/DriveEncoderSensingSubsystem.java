package org.firstinspires.ftc.teamcode.botcore.subsystems;

import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.framework.Subsystem;
import org.firstinspires.ftc.teamcode.botcore.subsystems.sensing.DriveEncoderSensing;

public class DriveEncoderSensingSubsystem extends Subsystem {

    DriveEncoderSensing driveEncoderSensing;


    public DriveEncoderSensingSubsystem(String name, BotManager botmgr, double rate) {
        super(name, botmgr, rate);


        driveEncoderSensing = new DriveEncoderSensing(botmgr, botmgr.getConfiguration().sensing.kEncoderInterval);

    }

    @Override
    public void execute() {

    }
}

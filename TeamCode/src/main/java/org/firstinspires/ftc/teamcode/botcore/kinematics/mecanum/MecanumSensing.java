package org.firstinspires.ftc.teamcode.botcore.kinematics.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.botcore.binding.SensorBindingBase;
import org.firstinspires.ftc.teamcode.botcore.binding.packages.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.framework.BotTaskManager;

public class MecanumSensing extends SensorBindingBase
{
    private DcMotor mMotor1, mMotor2, mMotor3, mMotor4;

    public MecanumSensing(BotTaskManager botmgr, long delay) {
        super(botmgr, delay);
    }

    @Override
    public void createSensors() {
        mMotor1 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor1);
        mMotor2 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor2);
        mMotor3 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor3);
        mMotor4 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor4);
    }

    @Override
    public MeasurementPackage sense() {

        MeasurementPackage mp = new MeasurementPackage();

        mp.mMotorEncoderVals = new int[4];

        mp.mMotorEncoderVals[0] = mMotor1.getCurrentPosition();
        mp.mMotorEncoderVals[1] = mMotor2.getCurrentPosition();
        mp.mMotorEncoderVals[2] = mMotor3.getCurrentPosition();
        mp.mMotorEncoderVals[3] = mMotor4.getCurrentPosition();

        return mp;
    }
}

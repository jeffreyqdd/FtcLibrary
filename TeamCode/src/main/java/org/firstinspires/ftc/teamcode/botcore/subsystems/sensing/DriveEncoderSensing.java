package org.firstinspires.ftc.teamcode.botcore.subsystems.sensing;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.botcore.binding.SensorBindingBase;
import org.firstinspires.ftc.teamcode.botcore.framework.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;

public class DriveEncoderSensing extends SensorBindingBase
{
    private DcMotor mMotor1, mMotor2, mMotor3, mMotor4;

    public DriveEncoderSensing(BotManager botmgr, long delay) {
        super(botmgr, delay);

    }

    @Override
    public void createSensors() {
        mMotor1 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor1);
        mMotor2 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor2);
        mMotor3 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor3);
        mMotor4 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor4);

        mMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void sense(MeasurementPackage measurementPackage) {


        measurementPackage.motorEncoderVals[0] = mMotor1.getCurrentPosition();
        measurementPackage.motorEncoderVals[1] = mMotor2.getCurrentPosition();
        measurementPackage.motorEncoderVals[2] = mMotor3.getCurrentPosition();
        measurementPackage.motorEncoderVals[3] = mMotor4.getCurrentPosition();

    }


}

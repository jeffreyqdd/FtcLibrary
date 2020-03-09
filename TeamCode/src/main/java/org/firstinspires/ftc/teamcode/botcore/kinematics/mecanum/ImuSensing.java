package org.firstinspires.ftc.teamcode.botcore.kinematics.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.botcore.binding.SensorBindingBase;
import org.firstinspires.ftc.teamcode.botcore.binding.packages.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.framework.BotTaskManager;

public class ImuSensing extends SensorBindingBase {

    private BNO055IMU mImu;

    public ImuSensing(BotTaskManager botmgr, long delay) {
        super(botmgr, delay);
    }

    @Override
    public void createSensors() {
        mImu = opmode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        mImu.initialize(parameters);
    }

    @Override
    public MeasurementPackage sense() {
        MeasurementPackage mp = new MeasurementPackage();
        mp.mOrientationInRadians = mImu.getAngularOrientation().firstAngle;

        return mp;
    }
}

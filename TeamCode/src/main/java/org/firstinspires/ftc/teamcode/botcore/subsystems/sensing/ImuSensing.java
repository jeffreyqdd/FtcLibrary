package org.firstinspires.ftc.teamcode.botcore.subsystems.sensing;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.botcore.binding.SensorBindingBase;
import org.firstinspires.ftc.teamcode.botcore.framework.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;

public class ImuSensing extends SensorBindingBase {

    private BNO055IMU mImu;

    public ImuSensing(BotManager botmgr, long delay) {
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
    public void sense(MeasurementPackage measurementPackage) {
        measurementPackage.orientationInRadians = mImu.getAngularOrientation().firstAngle;
    }


}

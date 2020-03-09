package org.firstinspires.ftc.teamcode.botcore.kinematics.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.botcore.binding.ActuatorBindingBase;
import org.firstinspires.ftc.teamcode.botcore.framework.BotTaskManager;

import java.util.List;

class MecanumActuation extends ActuatorBindingBase {

    private DcMotor mMotor1, mMotor2, mMotor3, mMotor4;


    public MecanumActuation(BotTaskManager botmgr) {
        super(botmgr);
    }

    @Override
    public void createActuators() {
        mMotor1 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor1);
        mMotor2 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor2);
        mMotor3 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor3);
        mMotor4 = opmode.hardwareMap.get(DcMotor.class, config.kinematic.kMotor4);

        mMotor1.setDirection(config.kinematic.kDirection1);
        mMotor2.setDirection(config.kinematic.kDirection2);
        mMotor3.setDirection(config.kinematic.kDirection3);
        mMotor4.setDirection(config.kinematic.kDirection4);

        mMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mMotor1.setMode(config.kinematic.kMode1);
        mMotor2.setMode(config.kinematic.kMode2);
        mMotor3.setMode(config.kinematic.kMode3);
        mMotor4.setMode(config.kinematic.kMode4);

        mMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void actuate(List<Double> power) {
        mMotor1.setPower(power.get(0));
        mMotor2.setPower(power.get(1));
        mMotor3.setPower(power.get(2));
        mMotor4.setPower(power.get(3));
    }

}

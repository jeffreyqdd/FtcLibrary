package org.firstinspires.ftc.teamcode.botcore.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.framework.Subsystem;
import org.firstinspires.ftc.teamcode.botcore.kinematics.MecanumBase;

public class MecanumChassisSubsystem extends Subsystem {

    MecanumBase mecanum;

    public MecanumChassisSubsystem(String name, BotManager botmgr, double rate) {
        super(name, botmgr, rate);

        mecanum = new MecanumBase(botmgr);


        Trajectory traj1 = mecanum.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(100)
                .forward(100)
                .build();
        Trajectory traj2 = mecanum.trajectoryBuilder(new Pose2d(0,200,0))
                .back(200)
                .build();

        mecanum.queueTrajectory(traj1);
        mecanum.queueTrajectory(traj2);
    }

    @Override
    public void execute() {

        //mecanum.tickWhileBusy();
        mecanum.setMotorPowers(0,0,0, botManager.getMeasurementPackage().gamepad1.leftStickY + botManager.getMeasurementPackage().gamepad1.rightStickY);
    }
}

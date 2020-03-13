package org.firstinspires.ftc.teamcode.botcore.kinematics;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.kinematics.actuation.DriveActuation;
import org.firstinspires.ftc.teamcode.botcore.utilities.DashboardUtil;
import org.firstinspires.ftc.teamcode.botcore.utilities.LynxModuleUtil;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class MecanumBase extends MecanumDrive {

    private BotManager botmgr;
    private BotConfiguration config;
    private DriveActuation mDrive;
    private OpMode mOpmode;



    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }


    //required for roadrunner, in a different naming convention

    public PIDCoefficients translationalPID, headingPID;
    private FtcDashboard dashboard;
    private NanoClock clock;
    private Mode mode;
    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;
    private DriveConstraints constraints;
    private TrajectoryFollower follower;
    private List<Pose2d> poseHistory;
    private List<Trajectory> queuedTrajectories;




    public MecanumBase(BotManager botmgr) {

        super(  botmgr.getConfiguration().kinematic.kV,
                botmgr.getConfiguration().kinematic.kA,
                botmgr.getConfiguration().kinematic.kStatic,
                botmgr.getConfiguration().kinematic.kLengthBetweenFrontWheels,
                botmgr.getConfiguration().kinematic.kLengthBetweenFrontAndRearWheels,
                botmgr.getConfiguration().kinematic.kSlideRatio
        );

        this.botmgr = botmgr;
        this.config = this.botmgr.getConfiguration();

        mDrive = new DriveActuation(this.botmgr);

        mOpmode = this.botmgr.getOpMode();


        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        clock = NanoClock.system();
        mode = Mode.IDLE;



        translationalPID = new PIDCoefficients(
                config.kinematic.kTranslationalP,
                config.kinematic.kTranslationalI,
                config.kinematic.kTranslationalD
                );


        headingPID = new PIDCoefficients(
                config.kinematic.kRotationalP,
                config.kinematic.kRotationalI,
                config.kinematic.kRotationalD
        );


        turnController = new PIDFController(headingPID);
        turnController.setInputBounds(0, 2 * Math.PI);


        constraints = new MecanumConstraints(config.kinematic.kBaseConstraints, config.kinematic.kLengthBetweenFrontWheels);
        follower = new HolonomicPIDVAFollower(translationalPID, translationalPID, headingPID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        poseHistory = new ArrayList<>();


        queuedTrajectories = new LinkedList<>();


        LynxModuleUtil.ensureMinimumFirmwareVersion(mOpmode.hardwareMap);
        for (LynxModule module : mOpmode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }


    public void tickWhileBusy() {
        if (isBusy()) {
            update();
        }
        else if(queuedTrajectories.size() > 0)
        {
            Trajectory next = queuedTrajectories.remove(0);
            followTrajectory(next);
        }
    }

    public void queueTrajectory(Trajectory traj)
    {
        queuedTrajectories.add(traj);
    }




    private void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    private void turn(double angle) {
        turnAsync(angle);
    }

    private void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    private void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
    }

    private Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }


    private void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }






    private boolean isBusy() {
        return mode != Mode.IDLE;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {

        List<Double> ret = new LinkedList<>();
        for(int i = 0; i < 4; i++)
        {
            ret.add(config.kinematic.encoderTicksToInches(botmgr.getMeasurementPackage().motorEncoderVals[i]));
        }

        return ret;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {

        List<Double> powers = new LinkedList<>();
        powers.add(v);
        powers.add(v1);
        powers.add(v2);
        powers.add(v3);

        mDrive.actuate(powers);
    }

    @Override
    protected double getRawExternalHeading() {
        return botmgr.getMeasurementPackage().orientationInRadians;
    }
}

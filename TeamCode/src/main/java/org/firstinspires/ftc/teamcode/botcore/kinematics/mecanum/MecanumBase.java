package org.firstinspires.ftc.teamcode.botcore.kinematics.mecanum;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;

import org.firstinspires.ftc.teamcode.botcore.binding.packages.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.framework.BotTaskManager;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;
import java.util.List;

public class MecanumBase extends MecanumDrive {

    private BotTaskManager mBotmgr;
    private BotConfiguration mConfig;
    private MecanumActuation mDrive;
    private MecanumSensing mSense;
    private ImuSensing mDirection;


    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    /*private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;*/



    public MecanumBase(BotTaskManager botmgr) {

        super(  botmgr.getConfig().kinematic.kV,
                botmgr.getConfig().kinematic.kA,
                botmgr.getConfig().kinematic.kStatic,
                botmgr.getConfig().kinematic.kLengthBetweenFrontWheels,
                botmgr.getConfig().kinematic.kLengthBetweenFrontWheels,
                botmgr.getConfig().kinematic.kSlideRatio
        );

        this.mBotmgr = botmgr;
        this.mConfig = mBotmgr.getConfig();

        mDrive = new MecanumActuation(mBotmgr);
        mSense = new MecanumSensing(mBotmgr, 20);
        mDirection = new ImuSensing(mBotmgr, 20);








        /*
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoeffici
       */















    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        MeasurementPackage mp = mSense.getReading();

        List<Double> ret = new LinkedList<>();

        for(int i = 0; i < 4; i++)
        {
            ret.add(mConfig.kinematic.encoderTicksToInches(mp.mMotorEncoderVals[i]));
        }
        return ret;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        LinkedList<Double> powers = new LinkedList<>();
        powers.add(v);
        powers.add(v1);
        powers.add(v2);
        powers.add(v3);
        mDrive.actuate(powers);
    }

    @Override
    protected double getRawExternalHeading() {
        return mDirection.getReading().mOrientationInRadians;
    }
}

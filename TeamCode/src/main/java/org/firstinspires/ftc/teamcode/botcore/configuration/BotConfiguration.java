package org.firstinspires.ftc.teamcode.botcore.configuration;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BotConfiguration
{

	public Kinematic kinematic;
	public Sensing sensing;

	public class Sensing
	{
		//in ms
		public final long kEncoderInterval = 50;
		public final long kImuInterval = 50;
	}

	public class Kinematic
	{
		//=====================================================
		//motor information
		//may need to change based off newer bindings, gearboxes, or motors.
		//=====================================================
		//motor names
		public final String kMotor1 = "top left drive";
		public final String kMotor2 = "bottom left drive";
		public final String kMotor3 = "bottom right drive";
		public final String kMotor4 = "top right drive";

		//motor modes
		public final DcMotor.RunMode kMode1 = DcMotor.RunMode.RUN_USING_ENCODER;
		public final DcMotor.RunMode kMode2 = DcMotor.RunMode.RUN_USING_ENCODER;
		public final DcMotor.RunMode kMode3 = DcMotor.RunMode.RUN_USING_ENCODER;
		public final DcMotor.RunMode kMode4 = DcMotor.RunMode.RUN_USING_ENCODER;

		//motor directions
		public final DcMotor.Direction kDirection1 = DcMotor.Direction.FORWARD;
		public final DcMotor.Direction kDirection2 = DcMotor.Direction.REVERSE;
		public final DcMotor.Direction kDirection3 = DcMotor.Direction.REVERSE;
		public final DcMotor.Direction kDirection4 = DcMotor.Direction.FORWARD;

		//motor values
		public final int kEncoderTicksPerRev = 560;
		public final int kRevolutionsPerMinute = 312;

		//=====================================================
		//chassis information
		//may need to change based off newer chassis or wheels
		//roadrunner requires that everything is imperial (inches)
		//=====================================================

		//wheel values
		public final double kWheelRadius = 1.9685;
		
		//kinematic values
		public final double kLengthBetweenFrontWheels = 16;
		public final double kLengthBetweenFrontAndRearWheels = 17;



		//=====================================================
		//pre-calculated values;
		//DO NOT TOUCH WITHOUT GOOD REASON
		//=====================================================


		public final double kMaxVel = ((double) kRevolutionsPerMinute / 60.0) * (kWheelRadius * 2.0 * Math.PI);


		//=====================================================
		//preset values:
		//DO NOT TOUCH WITHOUT GOOD REASON
		//=====================================================

		//kSlideRatio: how far sideways in one wheel rotation compared to forward
		public final double kSlideRatio = 1.0;
		public final double kV = 1.0 / kMaxVel;
		public final double kA = 0;
		public final double kStatic = 0;


		//=====================================================
		//PID values
		//may need to change based off newer chassis or wheels
		//tuning is very important!!!
		//=====================================================

		public final double kTranslationalP = 1;
		public final double kTranslationalI = 0;
		public final double kTranslationalD = 0;

		public final double kRotationalP = 1;
		public final double kRotationalI = 0;
		public final double kRotationalD = 0;

		public final DriveConstraints kBaseConstraints = new DriveConstraints(
				kMaxVel, kMaxVel * 0.2, 0.0,
				Math.toRadians(180.0), Math.toRadians(180.0) * 0.2, 0.0
		);



		//=====================================================
		//Important methods associated with drive train kinematics
		//DO NOT TOUCH WITHOUT GOOD REASON
		//=====================================================
		public double encoderTicksToInches(int ticks)
		{
			return kWheelRadius * 2 * Math.PI * ticks / kEncoderTicksPerRev;
		}
		public double encoderTicksToInches(double ticks)
		{
			return kWheelRadius * 2 * Math.PI * ticks / kEncoderTicksPerRev;
		}
	}

	public BotConfiguration(){
		kinematic = new Kinematic();
		sensing = new Sensing();
	}
}

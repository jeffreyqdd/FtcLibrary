package org.firstinspires.ftc.teamcode.botcore.binding.packages;

public class MeasurementPackage
{
	public int[] mMotorEncoderVals;
	public double mOrientationInRadians;

	public long TIMESTAMP;
	
	public MeasurementPackage()
	{
		TIMESTAMP = System.currentTimeMillis();
	}
	
}

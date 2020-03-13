package org.firstinspires.ftc.teamcode.botcore.framework;

public class MeasurementPackage
{
	public int[] motorEncoderVals;
	public double orientationInRadians;

	public Gamepad gamepad1, gamepad2;


	public class Gamepad
	{
		//triggers
		public double leftTrigger, rightTrigger;


		//bumpers
		public double leftBumper, rightBumper;

		//dpad
		public double dpadUp, dpadDown, dpadLeft, dpadRight;

		//buttons
		public double buttonA, buttonB, buttonX, buttonY;

		//sticks
		public double leftStickX, leftStickY;
		public double rightStickX, rightStickY;

		private Gamepad()
		{
			leftTrigger = 0.0;
			rightTrigger = 0.0;

			leftBumper = 0.0;
			rightBumper = 0.0;
			dpadUp = 0.0;
			dpadDown = 0.0;
			dpadLeft = 0.0;
			dpadRight = 0.0;

			buttonA = 0.0;
			buttonB = 0.0;
			buttonX = 0.0;
			buttonY = 0.0;
			leftStickX = 0.0;
			leftStickY = 0.0;
			rightStickX = 0.0;
			rightStickY = 0.0;

		}

	}

	public MeasurementPackage()
	{
		motorEncoderVals = new int[4];
		orientationInRadians = 0.0;

		gamepad1 = new Gamepad();
		gamepad2 = new Gamepad();


	}
	
}

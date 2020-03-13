package org.firstinspires.ftc.teamcode.botcore.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.utilities.LogUtils;

import java.util.HashSet;
import java.util.logging.Logger;

public class BotManager
{
	
	/* This block only needs to be invoked once for the entire project*/
	static {
		LogUtils.init();
	}
	// this logger needs to be created per class
	Logger logger = LogUtils.getLogger(BotManager.class.getName());
	

	//create bot configuration as well as the tasks
	private final BotConfiguration config;
	private final LinearOpMode opMode;
	private MeasurementPackage measurementPackage;
	private HashSet<Subsystem> subsystems;

	public BotManager(LinearOpMode opMode) {
		super();
		
		this.config = new BotConfiguration();
		this.opMode = opMode;
		this.measurementPackage = new MeasurementPackage();
		this.subsystems = new HashSet<>();
	}

	public LinearOpMode getOpMode()
	{
		return opMode;
	}

	public BotConfiguration getConfiguration()
	{
		return config;
	}
	public MeasurementPackage getMeasurementPackage() {
		 return measurementPackage;
	}

	public void addSubsystem(Subsystem s)
	{
		subsystems.add(s);
	}

	public void run()
	{
		while(opMode.opModeIsActive())
		{
			runSubsystems();
		}
	}
	private void runSubsystems()
	{
		for(Subsystem s : subsystems)
		{
			s.checkTick(System.currentTimeMillis());
		}
	}






}

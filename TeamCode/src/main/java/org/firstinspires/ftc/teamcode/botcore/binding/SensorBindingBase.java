package org.firstinspires.ftc.teamcode.botcore.binding;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.botcore.framework.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.utilities.LogUtils;

import java.util.logging.Logger;

public abstract class SensorBindingBase implements Sensor
{
	Logger logger = LogUtils.getLogger(SensorBindingBase.class.getName());

	private long delay;
	protected BotManager botmgr;
	protected BotConfiguration config;
	protected LinearOpMode opmode;
	protected MeasurementPackage measurementPackage;

	public SensorBindingBase(BotManager botmgr, long delay)
	{
		super();
		this.botmgr = botmgr;
		this.config = botmgr.getConfiguration();
		this.delay = delay;
		this.opmode = botmgr.getOpMode();
		this.measurementPackage = botmgr.getMeasurementPackage();

		createSensors();
		Thread sensingThread = new SensingThread();
		sensingThread.start();
		
	}
	
	@Override
	public MeasurementPackage getReading() {
		return measurementPackage;
	}
	
	
	public abstract void createSensors();
	public abstract void sense(MeasurementPackage measurementPackage);
	
	public class SensingThread extends Thread
	{
		@Override
		public void run()
		{
			try
			{
				while(!isInterrupted())
				{
					sense(measurementPackage);
					Thread.sleep(delay);
				}
			}
			catch (InterruptedException e) {logger.fine(this.getName() + " get interrupted");}
			catch (Exception e) {e.printStackTrace();}
		}
	}
	
	
}

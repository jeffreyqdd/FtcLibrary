package org.firstinspires.ftc.teamcode.botcore.binding;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.botcore.binding.packages.MeasurementPackage;
import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.framework.BotTaskManager;
import org.firstinspires.ftc.teamcode.botcore.utilities.LogUtils;

import java.util.logging.Logger;

public abstract class SensorBindingBase implements Sensor
{
	Logger logger = LogUtils.getLogger(SensorBindingBase.class.getName());
	
	private MeasurementPackage measurement;
	private long delay;
	protected BotTaskManager botmgr;
	protected BotConfiguration config;
	protected OpMode opmode;

	public SensorBindingBase(BotTaskManager botmgr, long delay)
	{
		super();
		this.measurement = null;
		this.botmgr = botmgr;
		this.config = botmgr.getConfig();
		this.delay = delay;
		this.opmode = botmgr.getOpMode();
		
		
		Thread sensingThread = new SensingThread();
		sensingThread.start();
		
	}
	
	@Override
	public MeasurementPackage getReading() {
		return measurement;
	}
	
	
	public abstract void createSensors();
	public abstract MeasurementPackage sense();
	
	public class SensingThread extends Thread
	{
		@Override
		public void run()
		{
			try
			{
				while(!isInterrupted())
				{
					measurement = sense();
					Thread.sleep(delay);
					//System.out.println("sensing");
				}
			}
			catch (InterruptedException e) {logger.fine(this.getName() + " get interrupted");}
			catch (Exception e) {e.printStackTrace();}
		}
	}
	
	
}

package org.firstinspires.ftc.teamcode.botcore.binding;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.framework.BotTaskManager;
import org.firstinspires.ftc.teamcode.botcore.utilities.LogUtils;

import java.util.List;
import java.util.logging.Logger;

public abstract class ActuatorBindingBase implements Actuator
{
	Logger logger = LogUtils.getLogger(SensorBindingBase.class.getName());
	
	protected BotTaskManager botmgr;
	protected BotConfiguration config;
	protected OpMode opmode;
	
	public ActuatorBindingBase(BotTaskManager botmgr)
	{
		super();

		this.botmgr = botmgr;
		this.config = botmgr.getConfig();
		this.opmode = botmgr.getOpMode();
		
		createActuators();
	}
	
	public abstract void createActuators();
	
	@Override
	public abstract void actuate(List<Double> values);
}

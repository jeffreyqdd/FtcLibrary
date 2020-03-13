package org.firstinspires.ftc.teamcode.botcore.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.botcore.configuration.BotConfiguration;
import org.firstinspires.ftc.teamcode.botcore.utilities.LogUtils;

import java.text.SimpleDateFormat;
import java.util.*;
import java.util.logging.Logger;

public abstract class Subsystem
{
	Logger logger = LogUtils.getLogger(Subsystem.class.getName());
	
	private final double rate;
	private final String name;
	private long prevTickTime;

	protected final BotManager botManager;
	protected final LinearOpMode opmode;
	protected final BotConfiguration config;

	

	
	public Subsystem(String name, BotManager botmgr, double rate) {
		super();
		this.rate = rate;
		this.botManager = botmgr;
		this.config = botmgr.getConfiguration();
		this.name = name;
		this.opmode = botmgr.getOpMode();
		prevTickTime = -1;
	}
	
	public BotManager getBotManager()
	{
		return botManager;
	}

	public LinearOpMode getOpmode() {
		return opmode;
	}

	public BotConfiguration getConfig() {
		return config;
	}

	public String getName() {
		return name;
	}


	public int getIntervalTimeMillis() {
		if (rate > 0)
			return (int) (1000 / rate );
		else
			return 50; // DEFAULT, slowest one takes this rate. (20 hz)
	}

	public void checkTick(long time) {

		if (time - prevTickTime >= getIntervalTimeMillis()) {
			execute();
			prevTickTime = time;
		}
	}

	/*
	 * without this, the task will not run
	 */
	public abstract void execute();
	
	
	@Override
	public String toString()
	{
		return name + " at " + rate + " hz";
	}
}

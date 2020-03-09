package org.firstinspires.ftc.teamcode.botcore.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.botcore.binding.ActuatorBindingBase;
import org.firstinspires.ftc.teamcode.botcore.binding.SensorBindingBase;
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
	protected final BotTaskManager botManager;
	protected final OpMode opmode;
	protected final BotConfiguration config;
	private final Set<TickType> ttoi;
	final Queue<TickEvent> mailbox = new LinkedList<>();
	
	// events buffer. waiting to be sent out per an explict WAIT_FOR_xxx event
	private Map<TickType, TickEvent> deferredEvents;
	
	//optional
	protected SensorBindingBase sensorBinding;
	protected ActuatorBindingBase actuatorBinding;
	
	
	public Subsystem(String name, BotTaskManager botmgr, double rate) {
		super();
		this.rate = rate;
		this.botManager = botmgr;
		this.config = botmgr.getConfig();
		this.name = name;
		this.opmode = botmgr.getOpMode();
		// instead of using system current. this allows using special wall clock
		prevTickTime = -1;
		
		ttoi = new HashSet<TickType>();
		deferredEvents = new HashMap<>();
		
		// this is to enforce the registration so it won't get missed by
		// children classes
		registerTickTypeOfInterest();
	}
	
	public void sendEvent(TickEvent tickEvent) {
		try {
			mailbox.add(tickEvent);
		}
		catch(Exception e) {
			logger.severe(e.getMessage());
		}
	}
	
	
	
	public boolean isTickTypeOfInterest(TickType type) {
		return ttoi.contains(type);
	}
	
	public double getRate() {
		return rate;
	}
	
	public int getIntervalTimeMillis() {
		if (rate > 0)
			return (int) (1000 / rate );
		else
			return 50; // DEFAULT, slowest one takes this rate.
	}
	
	public BotTaskManager getBotManager() {
		return botManager;
	}
	
	public String getName() {
		return name;
	}
	
	private String timeToString(long t) {
		SimpleDateFormat formatter= new SimpleDateFormat("HH:mm:ss.SSS");
		Date date = new Date(t);
		return formatter.format(date);
	}
	
	
	public Set<TickType> getTtoi() {
		return ttoi;
	}
	
	/**
	 * this is a signal from the manager that this is an opportunity to run.
	 * Expect this one to be overloaded so it can handle more tick types than
	 * just clock update. Please use super.tick() first to
	 *
	 */
	public void tick(TickEvent tk) {
		
		logger.finest("received tick events:" + tk);
		
		
		// check is the interval reached for sheduling based.
		if (tk.type == TickType.CLOCK_UPDATED) {
			
			if (prevTickTime < 0)
				prevTickTime = tk.ts;
			
			long currentTickTime = tk.ts;
			
			// IF the current task is scheduler based.
			long passedTime = currentTickTime - prevTickTime;
			
			if (passedTime >= getIntervalTimeMillis()) {
				// monitor overall performance
				if (passedTime > 2*getIntervalTimeMillis() ) {
					logger.warning("system slowed down. last execution is " + passedTime + "; should be around " + getIntervalTimeMillis() );
				}
				
				logger.finer("NonBlockingTask.tick: " + timeToString(currentTickTime)+ ":" + getName()+ " execute start");
				
				prevTickTime = currentTickTime;
			}
			else {
				return; // wait for next clock-update. too soon... I don't need the execution window
			}
		}
		
		// process all event in the mailbox
		
		
		// forward clock-update if needed
		if (getTtoi().contains(TickType.CLOCK_UPDATED)) {
			logger.fine(getName()+ "forward clock_update to inner logic per the registered interest");
			execute(tk);
		}
		
		int numOfMails = mailbox.size();
		for (int i=0; i< numOfMails; i++) {
			
			TickEvent eventInMail = null;
			
			try {
				eventInMail = mailbox.remove();
			}
			catch (Exception e) {
				logger.severe(getName() + " cannot remove message from mailbox though it says mail number is " + numOfMails+ e.getMessage());
			}
			
			if (eventInMail != null) {
				
				// check first is the tick of interest. this is a safe-guard because
				// the task manager should not dispatch such of-no-interest
				if (!getTtoi().contains(eventInMail.type)) {
					logger.severe(getName()+ "the taskmanager shall not send this ticktype to me: " + eventInMail.type + " ;expecting" + getTtoi());
					return;
				}
				
				logger.fine("start processing received event: " + eventInMail);
				// execute only for interested event. interval has arrived for scheduling based
				long startEventProc = getBotManager().getClock().currentTimeMillis();
				execute(eventInMail);
				long elapsed = getBotManager().getClock().currentTimeMillis() - startEventProc;
				
				if (elapsed > 20) {
					logger.warning(getName() + "LONGER_TIME in process event in mail." + eventInMail.type);
				}
				
			}
			
			
		}
	}
	/**
	 *
	 * @param deferred
	 * @return - false if rejected;
	 */
	public boolean addDeferredEvent(TickEvent deferred)
	{
		
		if (deferredEvents.containsKey(deferred.type))
		{
			return false;
		}
		
		deferredEvents.put(deferred.type, deferred);
		
		return true;
	}
	
	/**
	 * retrieve one per the type
	 * @param tickType
	 * @return null if not exist
	 */
	public TickEvent removeDeferredEvent(TickType tickType) {
		
		TickEvent o = deferredEvents.get(tickType);
		
		if (o != null)
			deferredEvents.remove(tickType);
		
		return  o;
	}
		
		
	/*
	 * Make the user register for it.
	 */
	public abstract void registerTickTypeOfInterest();
	
	
	/*
	 * without this, the task will not run
	 */
	public abstract void execute(TickEvent tk);
	
	
	
}

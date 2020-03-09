package org.firstinspires.ftc.teamcode.botcore.utilities;

import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.Handler;

public class LogUtils {
	
	public static void init() {
		
		setLevel(Level.SEVERE); // Level.SEVERE or Level.FINE
		
		//suppress
		setPackageLevel("org.glassfish", Level.WARNING);
		setPackageLevel(" com.qualcomm", Level.SEVERE);
		
	}
	
	public static Logger getLogger(String cname) {
		
		return Logger.getLogger(cname);
	}
	
	public static void setLevel(Level targetLevel) {
		Logger root = Logger.getLogger("");
		root.setLevel(targetLevel);
		for (Handler handler : root.getHandlers()) {
			handler.setLevel(targetLevel);
		}
		System.out.println("level set: " + targetLevel.getName());
	}
	
	
	public static void setPackageLevel(String packageName, Level targetLevel) {
		Logger root = Logger.getLogger(packageName);
		root.setLevel(targetLevel);
		for (Handler handler : root.getHandlers()) {
			handler.setLevel(targetLevel);
		}
		System.out.println("level set: " + targetLevel.getName());
	}
	
}
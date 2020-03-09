package org.firstinspires.ftc.teamcode.botcore.framework;

public enum TickType {
	// a regular clock update at 1.5 * max task freq. say tasks are in 50Hz, 20Hz, 1Hz.
	// the clock tick will be triggered at 75Hz
	CLOCK_UPDATED,
	LOCATION_UPDATED,
	;
	
}
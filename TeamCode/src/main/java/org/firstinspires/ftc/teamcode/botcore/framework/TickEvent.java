package org.firstinspires.ftc.teamcode.botcore.framework;

public class TickEvent {
	@Override
	public String toString() {
		return "TickEvent{" +
				"type=" + type +
				", body=" + body +
				", ts=" + ts +
				", deferResponse=" + deferResponse +
				'}';
	}
	
	public final TickType type;
	public final Object body;
	public long     ts = -1;  // not set. allow later multicaster to set it.
	public boolean deferResponse;
	
	public boolean isNoReceiverOk = false;
	
	
	public TickEvent(TickType type) {
		
		this(type,-1, null);
	}
	
	
	public TickEvent(TickType type, Object body) {
		
		this(type,-1, body);
	}
	
	
	public TickEvent(TickType type, long ts, Object body) {
		this(type, ts, body, false);
	}
	
	public TickEvent(TickType type, long ts, Object body, boolean deferResponse) {
		this.type = type;
		this.ts = ts;
		this.body = body;
		this.deferResponse = deferResponse;
	}
	
	public void setNoReceiverOk(boolean noReceiverOk) {
		isNoReceiverOk = noReceiverOk;
	}
	
	public boolean isNoReceiverOk() {
		return isNoReceiverOk;
	}
}
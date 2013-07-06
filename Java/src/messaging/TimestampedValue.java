package messaging;

import java.util.Date;

/**
 * A wrapper around a value that keeps track of when the wrapper was first created
 * @author fit-pc
 *
 * @param <Value> the type of object stored inside
 */
public class TimestampedValue<Value> {
	private Value val;
	private Date timestamp; 
	
	public TimestampedValue(Value v){
		val = v;
		timestamp = new Date();
	}
	
	public Date getTimestamp(){
		return timestamp;
	}
	
	public Value getValue(){
		return val;
	}
}

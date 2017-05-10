package com.palyrobotics.frc2017.util.logger;

/**
 * Created by Nihar on 3/13/17.
 * Stores a String with the timestamp on construction
 * Allows data structures to sort the strings by timestamp
 * And then retrieve the String
 * Also automatically adds a newline to the end
 */
public class TimestampedString implements Comparable<TimestampedString> {
	private String mString;	// Message
	private long mTime;	// Timestamp in milliseconds

	/**
	 * Constructor
	 * @param string Message to add to logger
	 */
	public TimestampedString(String string) {
		mString = string;
		mTime = System.currentTimeMillis();
	}

	/**
	 * @return Time of construction
	 */
	public long getTimestamp() {
		return mTime;
	}

	/**
	 * @return Timestamp in seconds and message
	 */
	public String getTimestampedString() {
		return (mTime/1000)+": "+mString+"\n";
	}

	@Override
	public String toString() {
		return getTimestampedString();
	}

	/**
	 * @return Comparison of timestamps
	 */
	@Override
	public int compareTo(TimestampedString o) {
		return Long.compare(this.getTimestamp(), o.getTimestamp());
	}
}

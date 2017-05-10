package com.palyrobotics.frc2017.config.dashboard;

/**
 * Represents a KV pair to publish to the dashboard
 * @author Robbie Selwyn
 */
public class DashboardValue {

	private final String key;
	private String value;
	
	/**
	 * Key where the data can be found in the networktable.
	 * @param key: the key in the networktable. 
	 */
	public DashboardValue(String key) {
		this.key = key;
		value = "NO_ELEMENT";
	}

	public DashboardValue(String key, String initialValue) {
		this.key = key;
		this.value = initialValue;
	}
	
	/**
	 * Updates value to desired object.
	 * @param val
	 */
	public void updateValue(Object val) {
		this.value = val.toString();
	}
	
	/**
	 * Package available method for the manager to call when passed a dashboard value.
	 * @return the key
	 */
	String getKey() {
		return this.key;
	}
	
	/**
	 * Package available method for the manager to call when passed a dashboard value.
	 * @return the value
	 */
	String getValue() {
		return this.value;
	}
}

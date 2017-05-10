package com.palyrobotics.frc2017.config.dashboard;

import com.palyrobotics.frc2017.config.Gains;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Handles input of data into the dashboard
 * @author Robbie Selwyn
 */
public class DashboardManager {

	// Usage of cantable or not
	private boolean mEnableCANTable = false;

	// Allow motion profile gains to be modified over NT
	public final boolean kPidTuning = false;
	
	public static final String TABLE_NAME = "RobotTable";
	public static final String CAN_TABLE_NAME = "data_table";
	
	public NetworkTable mRobotTable;
	private NetworkTable mCanTable;
	
	// Singleton setup
	private static DashboardManager instance = new DashboardManager();
	public static DashboardManager getInstance() {
		return instance;
	}
	
	private DashboardManager() {}
	
	/**
	 * Initialize network tables
	 */
	public void robotInit() {
		try {
			this.mRobotTable = NetworkTable.getTable(TABLE_NAME);
			Gains.initNetworkTableGains();
			if (mEnableCANTable) {
				this.mCanTable = NetworkTable.getTable(CAN_TABLE_NAME);
				NetworkTable.setUpdateRate(.015);
			}
		}
		catch (UnsatisfiedLinkError e) {
			// Catch the error that occurs during unit tests.
		}
		catch (NoClassDefFoundError e) {
		}
	}
	
	/**
	 * Publishes a KV pair to the Robot Network Table.
	 * @param d	The dashboard value.
	 */
	public void publishKVPair(DashboardValue d) {
		if (mRobotTable == null) {
			try {
				this.mRobotTable = NetworkTable.getTable(TABLE_NAME);
			}
			catch (UnsatisfiedLinkError e) {
				// Block the error in a unit test and don't publish the value.
			}
			catch (NoClassDefFoundError e) {}
		}
		
		// If we are now connected
		if (mRobotTable != null) {
			mRobotTable.putString(d.getKey(), d.getValue());
		}
	}
	
	/**
	 * Publishes CAN updates to the CAN Network Table.
	 * @param string The dashboard value.
	 */
	public void updateCANTable(String string) {
		if (mEnableCANTable) {
			return;
		}
		if (mCanTable != null) {
			// dashboard is connected
			mCanTable.putString("status", string+"\n");
		} else {
			// try to reach it again
			try {
				this.mCanTable = NetworkTable.getTable(CAN_TABLE_NAME);
			}
			catch (UnsatisfiedLinkError e) {
			}
			catch (NoClassDefFoundError e) {}
		}
	}

	/**
	 * Start or stop sending cantable data
	 * @param start true if you want to start sending data
	 */
	public void toggleCANTable(boolean start) {
		if (start) {
			if (mCanTable != null) {
				mCanTable.putString("start", "true");
				mCanTable.putString("end", "false");
			}
		} else {
			if (mCanTable != null) {
				mCanTable.putString("start", "false");
				mCanTable.putString("end", "true");
			}
		}
	}
}

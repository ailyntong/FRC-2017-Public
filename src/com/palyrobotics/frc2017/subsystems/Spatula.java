package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.config.dashboard.DashboardManager;
import com.palyrobotics.frc2017.config.dashboard.DashboardValue;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * STEIK SPATULA
 * @author Ailyn Tong
 * Represents a "spatula" that stores gears and allows for passive scoring
 * Controlled by one DoubleSolenoid which toggles between UP and DOWN
 */
public class Spatula extends Subsystem implements SubsystemLoop {
	// Singleton setup
	private static Spatula instance = new Spatula();
	private SpatulaState mState = SpatulaState.UP;
	public static Spatula getInstance() {
		return instance;
	}
	
	// Determines spatula behavior
	public enum SpatulaState { UP, DOWN }
	
	private DoubleSolenoid.Value mOutput = DoubleSolenoid.Value.kOff;

	private DashboardValue mDv;
	
	/**
	 * Constructor
	 */
	private Spatula() {
		super("Spatula");
		// Instantiate dashboard value
		mDv = new DashboardValue("spatulastatus");
	}

	@Override
	public void start() {
	}

	@Override
	public void stop() {
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		switch (commands.wantedSpatulaState) {
		case UP:
			mOutput = DoubleSolenoid.Value.kReverse;
			mState = SpatulaState.UP;
			break;
		case DOWN:
			mOutput = DoubleSolenoid.Value.kForward;
			mState = SpatulaState.DOWN;
			break;
		}
		// Updates dashboard value accordingly
		mDv.updateValue(mOutput.toString() == "kReverse" ? "UP" : "DOWN");
		DashboardManager.getInstance().publishKVPair(mDv);
	}
	
	/**
	 * @return The current spatula output
	 */
	public DoubleSolenoid.Value getOutput() {
		return mOutput;
	}
	
	/**
	 * @return The current spatula state
	 */
	public SpatulaState getState() {
		return mState;
	}

	@Override
	public String getStatus() {
		return "Spatula State: " + mState;
	}
}

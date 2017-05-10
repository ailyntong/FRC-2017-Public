package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.config.dashboard.DashboardManager;
import com.palyrobotics.frc2017.config.dashboard.DashboardValue;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Represents the (nonexistent) flippers
 * @author Ailyn Tong
 * Consists of two rods for the purpose of wall alignment during gear scoring
 * Each rod is controlled independently by a DoubleSolenoid
 */
public class Flippers extends Subsystem implements SubsystemLoop {
	// Singleton setup
	private static Flippers instance = new Flippers();
	public static Flippers getInstance() {
		return instance;
	}
	
	/**
	 * Stores values for left and right flippers
	 */
	public static class FlipperSignal {
		public DoubleSolenoid.Value leftFlipper, rightFlipper;
		
		public FlipperSignal(DoubleSolenoid.Value leftFlipper, DoubleSolenoid.Value rightFlipper) {
			this.leftFlipper = leftFlipper;
			this.rightFlipper = rightFlipper;
		}
	}
	
	private FlipperSignal mFlipperSignal;
	
	private DashboardValue mDv;
	
	/**
	 * Constructor
	 */
	private Flippers() {
		super("Flippers");
		// Instantiate dashboard value
		mDv = new DashboardValue("flipperstatus");
	}

	@Override
	public void start() {
	}

	@Override
	public void stop() {
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		// Direct control
		mFlipperSignal = commands.wantedFlipperSignal;
		// Update dashboard value
		mDv.updateValue("NO FLIPPERS");
		DashboardManager.getInstance().publishKVPair(mDv);
	}

	/**
	 * @return The current flippers output
	 */
	public FlipperSignal getFlipperSignal() {
		return mFlipperSignal;
	}

	@Override
	public String getStatus() {
		return "";
	}

}

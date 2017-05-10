package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Bang bang turn angle routine
 * Cancels after 5 seconds
 * @author Robbie Selwyn
 */
public class BBTurnAngleRoutine extends Routine {
	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{drive};
	}
	
	private double mAngle;	// Relative setpoint in degrees
	
	private double mStartTime;	// Start time in milliseconds
	
	/*
	 * START = Set new turn angle setpoint
	 * TURNING = Waiting to reach setpoint
	 * TIMED_OUT = Time limit passed
	 * DONE = Reached target or not operating
	 */
	private enum GyroBBState {
		START, TURNING, TIMED_OUT, DONE
	}
	private GyroBBState mState = GyroBBState.START;
	
	/**
	 * Constructor
	 * @param angle Angle in degrees relative to current heading
	 */
	public BBTurnAngleRoutine(double angle) {
		this.mAngle = angle;
	}
	
	/**
	 * Register start time and stop drivetrain
	 */
	@Override
	public void start() {
		System.out.println("start bb turn angle");
		drive.setNeutral();
		mState = GyroBBState.START;
		mStartTime = System.currentTimeMillis();
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {	
		// Routine times out after 5 seconds
		if (mState != GyroBBState.TIMED_OUT && (System.currentTimeMillis() - mStartTime > 5000)) {
			System.err.println("Timed out!");
			mState = GyroBBState.TIMED_OUT;
		}
		switch(mState) {
		case START:	// Set drive setpoint
			System.out.println("Set setpoint: " + mAngle);
			drive.setTurnAngleSetpoint(mAngle);
			commands.wantedDriveState = Drive.DriveState.ON_BOARD_CONTROLLER;
			mState = GyroBBState.TURNING;
			break;
		case TURNING:
			if(drive.controllerOnTarget()) {
				mState = GyroBBState.DONE;
			}
			break;
		case TIMED_OUT:	// Stop drivetrain
			drive.setNeutral();
			break;
		case DONE:	// Cancel controller
			drive.resetController();
			break;
		}
		
		return commands;
	}
	
	/**
	 * Stop drivetrain
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		mState = GyroBBState.DONE;
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		drive.setNeutral();
		return commands;
	}

	/**
	 * @return Whether the routine is finished
	 */
	@Override
	public boolean finished() {
		return mState == GyroBBState.DONE;
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "BangBangGyroTurnAngleRoutine";
	}

}

package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Turns a specified angle using encoders
 * @author Eric Liu
 *
 */
public class EncoderTurnAngleRoutine extends Routine {
	
	private double mAngle;	// Relative setpoint in degrees
	
	/*
	 * START = Set new turn angle setpoint
	 * TURNING = Waiting to reach setpoint
	 * DONE = reached target or not operating
	 */
	private enum State {
		START, TURNING, DONE
	}
	private State mState = State.START;
	
	/**
	 * Constructor
	 * @param angle Angle in degrees relative to current heading
	 */
	public EncoderTurnAngleRoutine(double angle) {
		this.mAngle = angle;
	}
	
	/**
	 * Reset drivetrain
	 */
	@Override
	public void start() {
		drive.setNeutral();
		mState = State.START;
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		Commands output = commands.copy();
		switch(mState) {
		case START:
			drive.setTurnAngleEncoderSetpoint(mAngle);
			output.wantedDriveState = Drive.DriveState.OFF_BOARD_CONTROLLER;
			mState = State.TURNING;
			break;
		case TURNING:
			output.wantedDriveState = Drive.DriveState.OFF_BOARD_CONTROLLER;
			if(drive.controllerOnTarget() && drive.hasController()) {
				mState = State.DONE;
			}
			break;
		case DONE:
			drive.resetController();
			break;
		default:
			break;
		}
		return output;
	}

	/**
	 * Stop drivetrain
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		mState = State.DONE;
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		drive.setNeutral();
		return commands;
	}

	/**
	 * @return Whether the routine is finished
	 */
	@Override
	public boolean finished() {
		return mState == State.DONE;
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{drive};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "EncoderTurnAngleRoutine";
	}

}

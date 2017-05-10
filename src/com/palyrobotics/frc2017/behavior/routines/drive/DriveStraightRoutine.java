package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Routine that drives straight a set distance
 * @author Robbie Selwyn
 *
 */
public class DriveStraightRoutine extends Routine {
	private double mDistance;	// inches
	
	/**
	 * Constructor
	 * @param distance Distance in inches
	 */
	public DriveStraightRoutine(double distance) {
		this.mDistance = distance;
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{drive};
	}
	/*
	 * START = Set new drive setpoint
	 * DRIVING = Waiting to reach drive setpoint
	 * DONE = Reached target or not operating
	 */
	private enum DriveStraightRoutineState {
		START, DRIVING, DONE
	}

	DriveStraightRoutineState state = DriveStraightRoutineState.START;
	
	/**
	 * Reset drivetrain
	 */
	@Override
	public void start() {
		drive.setNeutral();
		state = DriveStraightRoutineState.START;
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		Commands output = commands.copy();
		switch(state) {
		case START:	// Set drive controller
			drive.setDriveStraight(mDistance);
			output.wantedDriveState = Drive.DriveState.ON_BOARD_CONTROLLER;
			state = DriveStraightRoutineState.DRIVING;
			break;
		case DRIVING:	// Update drive controller
			output.wantedDriveState = Drive.DriveState.ON_BOARD_CONTROLLER;
			if (drive.controllerOnTarget() && drive.hasController()) {
				state = DriveStraightRoutineState.DONE;
			}
			break;
		case DONE:	// Reset drive controller
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
		System.out.println("Cancelling DriveStraightRoutine");
		state = DriveStraightRoutineState.DONE;
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		drive.resetController();
		return commands;
	}

	/**
	 * @return Whether routine has finished
	 */
	@Override
	public boolean finished() {
		return state == DriveStraightRoutineState.DONE;
	}
	
	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "DriveStraightRoutine";
	}

}

package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.archive.DriveSignal;

import com.palyrobotics.frc2017.util.Subsystem;

import java.util.Optional;

/**
 * Drive at a specified output for a specified length of time
 * @author Nihar Mitra
 */
public class DriveTimeRoutine extends Routine {
	private long mEndTime;	// End time in milliseconds
	private DriveSignal mDrivePower;	// PercentVbus output (-1 to 1)

	/**
	 * Constructs with a specified time setpoint and velocity
	 * @param time How long to drive (seconds)
	 * @param drivePower LegacyDrive signal to output (left/right speeds -1 to 1)
	 */
	public DriveTimeRoutine(double time, DriveSignal drivePower) {
		// Keeps the offset prepared, when routine starts, will add System.currentTime
		mEndTime = (long) (1000*time);
		this.mDrivePower = drivePower;
	}

	/**
	 * Initialize end time, reset controller
	 */
	@Override
	public void start() {
		drive.resetController();
		// mEndTime already has the desired drive time
		mEndTime += System.currentTimeMillis();
	}

	//Routines just change the states of the robotsetpoints, which the behavior manager then moves the physical subsystems based on.
	/**
	 * Update setpoint
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		commands.wantedDriveState = Drive.DriveState.OPEN_LOOP;
		commands.robotSetpoints.drivePowerSetpoint = Optional.of(mDrivePower);
		return commands;
	}

	/**
	 * Stop drivetrain
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		System.out.println("Cancelling");
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		drive.resetController();
		drive.setNeutral();
		return commands;
	}

	/**
	 * @return True after the time is up
	 */
	@Override
	public boolean finished() {
		return (System.currentTimeMillis() >= mEndTime);
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "DriveTimeRoutine";
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{drive};
	}
}
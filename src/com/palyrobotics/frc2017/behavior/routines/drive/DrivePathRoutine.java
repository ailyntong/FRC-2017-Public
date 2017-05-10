package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.Subsystem;
import com.team254.lib.trajectory.Path;

/**
 * Created by Nihar on 4/5/17.
 */
public class DrivePathRoutine extends Routine {
	private Path mPath;	// Path to follow
	private Gains.TrajectoryGains mGains;	// Trajectory PID gains
	private boolean mUseGyro;	// Whether to correct variations using gyro
	private boolean mInverted;	// Whether to left/right invert the path
	
	/**
	 * Constructor
	 * @param path Path to follow
	 * @param gains Trajectory PID gains
	 * @param useGyro Whether to correct variations using gyro
	 * @param inverted Whether to left/right invert the path
	 */
	public DrivePathRoutine(Path path, Gains.TrajectoryGains gains, boolean useGyro, boolean inverted) {
		this.mPath = path;
		this.mGains = gains;
		this.mUseGyro = useGyro;
		this.mInverted = inverted;
	}

	/**
	 * Set drive controller
	 */
	@Override
	public void start() {
		drive.setTrajectoryController(mPath, mGains, mUseGyro, mInverted);
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		commands.wantedDriveState = Drive.DriveState.ON_BOARD_CONTROLLER;
		return commands;
	}

	/**
	 * Stop drivetrain
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		drive.setNeutral();
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		return commands;
	}

	/**
	 * @return Whether the controller is on target
	 */
	@Override
	public boolean finished() {
		return drive.controllerOnTarget();
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
		return "DrivePathRoutine"+((mUseGyro)?"gyro":"noGyro");
	}
}

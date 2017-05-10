package com.palyrobotics.frc2017.behavior.routines;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Routine that raises the spatula
 * @author Ailyn Tong
 */
public class SpatulaUpRoutine extends Routine {
	private double mStartTime = 0;	// So the routine doesn't cancel early
	private boolean mUpdated = false;	// Whether or not update() has been run at least once

	/**
	 * Initialize start time
	 */
	@Override
	public void start() {
		mStartTime = System.currentTimeMillis();
	}

	/**
	 * Raise spatula
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		mUpdated = true;
		commands.wantedSpatulaState = Spatula.SpatulaState.UP;
		return commands;
	}

	/**
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		return commands;
	}

	/**
	 * @return True after a time has passed
	 */
	@Override
	public boolean finished() {
		return mUpdated && (System.currentTimeMillis() - mStartTime) > 2000;
	}

	/**
	 * @return Set of all subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{Spatula.getInstance()};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "SpatulaUpRoutine";
	}

}

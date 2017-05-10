package com.palyrobotics.frc2017.behavior;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.util.Subsystem;

import java.util.ArrayList;

/**
 * Created by Nihar on 12/27/16.
 * A compound routine that runs multiple routines in parallel.
 */
public class ParallelRoutine extends Routine {
	private ArrayList<Routine> mRoutines;

	/**
	 * Runs all routines at the same time
	 * Finishes when all routines finish
	 * @param routines ArrayList of routines
	 */
	public ParallelRoutine(ArrayList<Routine> routines) {
		this.mRoutines = routines;
	}

	/**
	 * Start all routines
	 */
	@Override
	public void start() {
		for(Routine routine: mRoutines) {
			routine.start();
		}
	}

	/**
	 * Update all running routines
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		for(Routine routine: mRoutines) {
			if(!routine.finished()) {
				commands = routine.update(commands);
			}
		}
		return commands;
	}

	/**
	 * Cancel all routines
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		for(Routine routine : mRoutines) {
			commands = routine.cancel(commands);
		}
		return commands;
	}

	/**
	 * @return Whether or not all routines have finished
	 */
	@Override
	public boolean finished() {
		for(Routine routine : mRoutines) {
			if(!routine.finished()) {
				return false;
			}
		}
		return true;
	}

	/**
	 * @return Set of all subsystems used by routines
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return RoutineManager.sharedSubsystems(mRoutines);
	}

	/**
	 * @return Name of all routines
	 */
	@Override
	public String getName() {
		String name = "ParallelRoutine of (";
		for(Routine routine: mRoutines) {
			name += (routine.getName() + " ");
		}
		return name + ")";
	}
}

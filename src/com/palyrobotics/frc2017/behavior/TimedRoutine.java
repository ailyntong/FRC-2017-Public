package com.palyrobotics.frc2017.behavior;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.util.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Nihar on 12/27/16.
 * A routine that can run multiple routines in parallel, 
 * but will end after the time limit has passed
 */
public class TimedRoutine extends Routine {
	private Routine[] mRoutines;
	private double mTime, mStartTime;	// Milliseconds
	/**
	 * Constructor
	 * @param time Time in seconds before routine automatically finishes
	 * @param routines Routines to run in parallel
	 */
	public TimedRoutine(double time, Routine... routines) {
		mRoutines = routines;
		this.mTime = time*1000;
	}

	/**
	 * Register start time
	 */
	@Override
	public void start() {
		this.mStartTime = System.currentTimeMillis();
	}

	/**
	 * Update all routines
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		for (Routine r : mRoutines) {
			commands = r.update(commands);
		}
		return commands;
	}

	/**
	 * Cancel all routines
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		for (Routine r : mRoutines) {
			commands = r.cancel(commands);
		}
		return commands;
	}

	/**
	 * @return Whether the time limit has passed or all routines have finished
	 */
	@Override
	public boolean finished() {
		if (System.currentTimeMillis() > mStartTime + mTime) {
			System.out.println("Timed out routine");
			return true;
		}
		for (Routine r : mRoutines) {
			if (!r.finished()) {
				return false;
			}
		}
		return true;
	}

	/**
	 * @return Set of subsystems shared by all routines
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return RoutineManager.sharedSubsystems(new ArrayList<>(Arrays.asList(mRoutines)));
	}

	@Override
	public String getName() {
		String name = "(Timed"+mTime+"Routine of ";
		for (Routine r : mRoutines) {
			name += r.getName();
		}
		name+=")";
		return name;
	}
}

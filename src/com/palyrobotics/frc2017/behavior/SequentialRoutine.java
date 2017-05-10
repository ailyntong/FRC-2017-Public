package com.palyrobotics.frc2017.behavior;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.util.Subsystem;

import java.util.ArrayList;

/**
 * Created by Nihar on 12/27/16.
 * A compound routine that runs multiple routines in series
 */
public class SequentialRoutine extends Routine {
	private ArrayList<Routine> mRoutines;
	private int mRunningRoutineIndex = 0;	// Tracks index of current routine
	private boolean mIsDone = false;
	private Subsystem[] mRequiredSubsystems;

	/**
	 * Constructor
	 * @param routines Routines to run in series
	 */
	public SequentialRoutine(ArrayList<Routine> routines) {
		mRoutines = routines;
		mRequiredSubsystems = RoutineManager.sharedSubsystems(mRoutines);
	}

	/**
	 * Start current routine
	 */
	@Override
	public void start() {
		mRoutines.get(mRunningRoutineIndex).start();
	}

	/**
	 * Update current routine and move on to the next one if finished
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		Commands output = commands.copy();
		if(mIsDone) {
			return output;
		}
		// Update the current routine
		output = mRoutines.get(mRunningRoutineIndex).update(output);
		// Keep moving to next routine if the current routine is finished
		while(mRoutines.get(mRunningRoutineIndex).finished()) {
			output = mRoutines.get(mRunningRoutineIndex).cancel(output);
			if(mRunningRoutineIndex <= mRoutines.size()-1) {
				mRunningRoutineIndex++;
			}
			
			// If final routine is finished, don't update anything
			if(mRunningRoutineIndex > mRoutines.size()-1) {
				mIsDone = true;
				break;
			}
			
			// Start the next routine
			mRoutines.get(mRunningRoutineIndex).start();
		}
		return output;
	}

	/**
	 * Cancel current routine
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		//If not all routines finished, cancel the current routine. Otherwise everything is already finished.
		if(mRunningRoutineIndex < mRoutines.size()) {
			mRoutines.get(mRunningRoutineIndex).cancel(commands);
		}
		
		return commands;
	}

	/**
	 * @return Whether or not all routines have finished
	 */
	@Override
	public boolean finished() {
		return mIsDone;
	}

	/**
	 * @return Set of all subsystems used by routines
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return mRequiredSubsystems;
	}

	@Override
	public String getName() {
		String name = "SequentialRoutine of (";
		for(Routine routine : mRoutines) {
			name += (routine.getName() + " ");
		}
		return name + ")";
	}
}

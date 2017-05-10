package com.palyrobotics.frc2017.behavior.routines;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Routine that waits the specified amount of time
 * Does not require any subsystems
 * @author Nihar Mitra
 */
public class TimeoutRoutine extends Routine {
    double timeout;	// Wait time in seconds
    double timeStart;	// Start time in milliseconds

	/**
	 * Constructor
	 * @param waitTime time to wait in seconds
	 */
	public TimeoutRoutine(double waitTime) {
        this.timeout = waitTime;
    }

	/**
	 * @return Whether the specified time has passed
	 */
    @Override
    public boolean finished() {
        return System.currentTimeMillis() >= timeStart + 1000 * timeout;
    }

    /**
     * @return Modified commands
     */
    @Override
    public Commands update(Commands commands) {
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
     * Register start time
     */
    @Override
    public void start() {
        timeStart = System.currentTimeMillis();
    }

    /**
     * @return Empty array
     */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "TimeoutRoutine";
	}

}

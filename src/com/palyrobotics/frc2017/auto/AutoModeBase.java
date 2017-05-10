package com.palyrobotics.frc2017.auto;

import com.palyrobotics.frc2017.behavior.Routine;

/**
 * Template for autonomous modes, which are sequences of routines.
 * After initialization, the robot calls prestart() to generate a routine
 * that will then be added to the RoutineManager.
 * @author Nihar Mitra
 */
public abstract class AutoModeBase {
	protected boolean active = false;

	public abstract String toString();

	// Will be run before the routine is taken
	public abstract void prestart();

	public abstract Routine getRoutine();

	public void stop() {
		active = false;
	}

	public boolean isActive() {
		return active;
	}
}
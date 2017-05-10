package com.palyrobotics.frc2017.behavior.routines.scoring;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.subsystems.Spatula.SpatulaState;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Moves the slider to a setpoint
 * Slider target needs to be set from elsewhere
 * @deprecated, use AutocorrectPositioning
 * NOTE: When unit testing, set Robot.RobotState appropriately
 * @author Prashanti
 */
public class PositioningSliderRoutine extends Routine {
	// Whether this routine is allowed to run or not
	private boolean mAllowed;
	
	/**
	 * Determine if routine is allowed to run
	 */
	@Override
	public void start() {
		if (spatula.getState() == SpatulaState.DOWN) {
			mAllowed = false;
		} else {
			mAllowed = true;
		}
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		if (mAllowed) {
			commands.wantedSliderState = Slider.SliderState.AUTOMATIC_POSITIONING;

		} else {
			commands.wantedSliderState = Slider.SliderState.IDLE;
		}
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			System.err.println("Slider position routine rejected!");
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * Stop slider
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		commands.wantedSliderState = Slider.SliderState.IDLE;
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * @return Whether slider is on target or routine is not allowed
	 */
	@Override
	public boolean finished() {
		return !mAllowed || slider.onTarget();
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{Slider.getInstance(), Spatula.getInstance()};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "SliderDistancePositioningRoutine";
	}

}

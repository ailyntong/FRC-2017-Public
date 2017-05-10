package com.palyrobotics.frc2017.behavior.routines.scoring;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Slider.SliderState;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Allows for direct joystick control of slider
 * @author Prashanti Anderson
 */
public class ManualControlSliderRoutine extends Routine {	
	@Override
	public void start() {	
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		try {
			commands.wantedSliderState = SliderState.MANUAL;
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			System.err.println("Manual Slider Routine rejected!");
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
		commands.wantedSliderState = SliderState.IDLE;
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * Manual control continues until overridden by another routine
	 * @return false
	 */
	@Override
	public boolean finished() {
		return false;
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{Slider.getInstance()};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "ManualSliderControlRoutine";
	}

}
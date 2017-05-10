package com.palyrobotics.frc2017.behavior.routines;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.subsystems.Intake;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Routine that centers the slider (if necessary), then lowers the spatula
 * @author Ailyn Tong
 */
public class SpatulaDownAutocorrectRoutine extends Routine {
	// Slider must be centered before spatula can be lowered
	private enum AutocorrectState {
		CENTERING,
		FLIPPING
	}
	private AutocorrectState mState = AutocorrectState.CENTERING;
	
	private double mStartTime = 0;	// So the routine doesn't cancel early
	private boolean mUpdated = false;	// Whether or not update() has been run at least once
	
	/**
	 * Initialize start time and state
	 */
	@Override
	public void start() {
		// Register start time
		mStartTime = System.currentTimeMillis();
		// Skip to flipping if slider is centered otherwise, centering
		mState = (Math.abs(Robot.getRobotState().sliderEncoder) < 40) ? AutocorrectState.FLIPPING : AutocorrectState.CENTERING;
	}

	/**
	 * Update state machine
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		mUpdated = true;
//		commands.robotSetpoints.sliderSetpoint = Slider.SliderTarget.CENTER;
		switch (mState) {
		case CENTERING:	// Center the slider
			commands.robotSetpoints.sliderSetpoint = Slider.SliderTarget.CENTER;
			commands.wantedSliderState = Slider.SliderState.AUTOMATIC_POSITIONING;
			commands.wantedSpatulaState = Spatula.SpatulaState.UP;
			// Move on after slider stops moving
			if (System.currentTimeMillis()-mStartTime > 300 && Robot.getRobotState().sliderVelocity == 0) {
				mState = AutocorrectState.FLIPPING;
				break;
			}
			break;
		case FLIPPING:	// Lower the spatula
			commands.wantedSpatulaState = Spatula.SpatulaState.DOWN;
			break;
		}
		
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * Set all states to idle
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		commands.robotSetpoints.sliderSetpoint = Slider.SliderTarget.NONE;
		commands.wantedSliderState = Slider.SliderState.IDLE;
		commands.wantedIntakeState = Intake.IntakeState.IDLE;
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * @return Whether or not routine has finished properly
	 */
	@Override
	public boolean finished() {
		return mUpdated && mState == AutocorrectState.FLIPPING && (System.currentTimeMillis() - mStartTime) > 2000;
	}

	/**
	 * @return Set of all subsystems required by this routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{Slider.getInstance(), Spatula.getInstance(), Intake.getInstance()};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "SpatulaDownAutocorrectRoutine";
	}

}

package com.palyrobotics.frc2017.behavior.routines.scoring;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.subsystems.Slider.SliderState;
import com.palyrobotics.frc2017.subsystems.Spatula.SpatulaState;
import com.palyrobotics.frc2017.util.Subsystem;

/** 
 * Autocorrects -> only tells the slider to move once safe (spatula up)
 * @author Prashanti, Nihar, Ailyn
 */
public class AutocorrectPositioningSliderRoutine extends Routine {
	// Spatula needs to be raised before moving slider
	private enum DistancePositioningState {
		RAISING,
		MOVING
	}
	private DistancePositioningState mState = DistancePositioningState.RAISING;
	
	// Use to make sure routine ran at least once before "finished"
	private boolean mUpdated = false;
	
	private Slider.SliderTarget mTarget;	// Slider setpoint
	
	private double mStartTime;
	private static final double kRaiseTime = 1000;
	
	/**
	 * Constructor
	 * @param target Slider setpoint
	 */
	public AutocorrectPositioningSliderRoutine(Slider.SliderTarget target) {
		mTarget = target;
	}
	
	/**
	 * Set initial state and register start time
	 */
	@Override
	public void start() {
		if (spatula.getState() == SpatulaState.DOWN || slider.getSliderState() == Slider.SliderState.WAITING) {
			System.out.println("Autocorrecting spatula!");
			mState = DistancePositioningState.RAISING;
		}
		else {
			mState = DistancePositioningState.MOVING;
		}
		mStartTime = System.currentTimeMillis();
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		commands.robotSetpoints.sliderSetpoint = mTarget;
		mUpdated = true;
		switch(mState) {
		case MOVING:
			commands.wantedSliderState = Slider.SliderState.AUTOMATIC_POSITIONING;
			break;
		case RAISING:	// Wait for spatula to be fully raised before moving to next state
			if(System.currentTimeMillis() > (kRaiseTime+mStartTime)) {
				System.out.println("Time up");
				mState = DistancePositioningState.MOVING;
				break;
			}
			commands.wantedSpatulaState = Spatula.SpatulaState.UP;
			commands.wantedSliderState = Slider.SliderState.WAITING;
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
	 * @return Whether the slider is on target
	 */
	@Override
	public boolean finished() {
		return mUpdated && mState==DistancePositioningState.MOVING && slider.onTarget();
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
		return "SliderDistancePositioningAutocorrectRoutine";
	}

}

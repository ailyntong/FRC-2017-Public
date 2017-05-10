package com.palyrobotics.frc2017.behavior.routines.scoring;

import java.util.Optional;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.robot.HardwareAdapter;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.subsystems.Spatula.SpatulaState;
import com.palyrobotics.frc2017.util.Subsystem; 

/**
 * Moves the slider to a setpoint
 * NOTE: When unit testing, set Robot.RobotState appropriately
 * @author Prashanti
 */
public class CustomPositioningSliderRoutine extends Routine {
	// Spatula needs to be raised before moving slider
	private enum DistancePositioningState {
		RAISING,
		MOVING
	}
	private DistancePositioningState mState = DistancePositioningState.RAISING;
	
	// Use to make sure routine ran at least once before "finished"
	private boolean mUpdated = false;
	
	private double mTarget;	// Slider position in inches
	
	private double mStartTime;	// Milliseconds
	private static final double kRaiseTime = 1700;	// Milliseconds
	
	/**
	 * Constructor
	 * @param target Absolute slider position in inches
	 */
	public CustomPositioningSliderRoutine(double target) {
		this.mTarget = target;
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
		commands.robotSetpoints.sliderSetpoint = Slider.SliderTarget.CUSTOM;
		commands.robotSetpoints.sliderCustomSetpoint = Optional.of(mTarget * Constants.kSliderRevolutionsPerInch);
		mUpdated = true;
		switch(mState) {
		case MOVING:
			commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
			try {
				slider.run(commands, this);
			} catch (IllegalAccessException e) {
				e.printStackTrace();
//			} catch (InterruptedException e) {
//				e.printStackTrace();
			}
			break;
		case RAISING:	// Wait for spatula to be fully raised before moving to next state
			if(System.currentTimeMillis() > (kRaiseTime+mStartTime)) {
				mState = DistancePositioningState.MOVING;
				break;
			}
			commands.wantedSpatulaState = Spatula.SpatulaState.UP;
			commands.wantedSliderState = Slider.SliderState.WAITING;
			break;
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
		commands.robotSetpoints.sliderCustomSetpoint = Optional.empty();
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * @return Whether the slider is on target or the time limit has passed
	 */
	@Override
	public boolean finished() {
		RobotState robotState = Robot.getRobotState();

		if(!HardwareAdapter.getInstance().getSlider().sliderTalon.getControlMode().isPID()) {
			return false;
		}
		// Give up after 1.5 seconds
		if (System.currentTimeMillis()-mStartTime > 1500) {
			return true;
		}
		return mUpdated && mState==DistancePositioningState.MOVING &&
				(System.currentTimeMillis() - mStartTime > 1000) && (robotState.sliderVelocity == 0) && slider.onTarget();
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
		return "SliderDistanceCustomPositioningRoutine";
	}

}
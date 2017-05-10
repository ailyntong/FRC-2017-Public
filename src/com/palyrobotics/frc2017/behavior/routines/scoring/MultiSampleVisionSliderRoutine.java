package com.palyrobotics.frc2017.behavior.routines.scoring;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.vision.AndroidConnectionHelper;

import java.util.Arrays;
import java.util.Optional;

/**
 * Moves slider to 3 positions to sample vision data and finds the best target
 * before determining the slider position
 * @author Nihar, Alvin
 */
public class MultiSampleVisionSliderRoutine extends Routine {
	private enum SamplingState {
		LEFT, CENTER, RIGHT, SCORE
	}
	private SamplingState mState;
	
	private boolean mNewState = false;
	private double mStartTime = 0;	// Milliseconds
	private double[] mVisionSetpoints = new double[3];	// Slider positions in inches
	private double kThreshold = 1;	// Inches

	/**
	 * Register start time
	 */
	@Override
	public void start() {
		mState = SamplingState.LEFT;
		mStartTime = System.currentTimeMillis();
		mNewState = true;
	}

	/**
	 * Record vision setpoint at each of: left, center, right
	 * Then determine best setpoint
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		switch (mState) {
			case LEFT:
				if (mNewState) {
					commands.robotSetpoints.sliderCustomSetpoint = Optional.of(-7.0);
					commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
					mStartTime = System.currentTimeMillis();
					mNewState = false;
				}
				if (Robot.getRobotState().sliderVelocity==0 && System.currentTimeMillis()-mStartTime > 200) {
					mVisionSetpoints[0] = AndroidConnectionHelper.getInstance().getXDist();
					mState = SamplingState.CENTER;
					mNewState = true;
				}
				break;
			case CENTER:
				if (mNewState) {
					commands.robotSetpoints.sliderCustomSetpoint = Optional.of(0.0);
					commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
					mStartTime = System.currentTimeMillis();
					mNewState = false;
				}
				if (Robot.getRobotState().sliderVelocity==0 && System.currentTimeMillis()-mStartTime > 200) {
					mState = SamplingState.RIGHT;
					mVisionSetpoints[1] = AndroidConnectionHelper.getInstance().getXDist();
					mState = SamplingState.RIGHT;
					mNewState = true;
				}
				break;
			case RIGHT:
				if (mNewState) {
					commands.robotSetpoints.sliderCustomSetpoint = Optional.of(7.0);
					commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
					mStartTime = System.currentTimeMillis();
					mNewState = false;
				}
				if (Robot.getRobotState().sliderVelocity==0 && System.currentTimeMillis()-mStartTime > 200) {					mState = SamplingState.SCORE;
					mVisionSetpoints[2] = AndroidConnectionHelper.getInstance().getXDist();
					mState = SamplingState.SCORE;
					mNewState = true;
				}
				break;
			case SCORE:
				commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
				if (mNewState) {
					mNewState = false;
					Arrays.sort(mVisionSetpoints);
					mStartTime = System.currentTimeMillis();
					System.out.println("Vision setpoints: " + Arrays.toString(mVisionSetpoints));
					if (mVisionSetpoints[1] - mVisionSetpoints[0] < kThreshold) {
						double setpoint = (mVisionSetpoints[1] + mVisionSetpoints[0]) / 2;
						System.out.println("Chosen one: " + setpoint);
						commands.robotSetpoints.sliderCustomSetpoint = Optional.of(setpoint);
						break;
					}
					else if (mVisionSetpoints[2] - mVisionSetpoints[1] > kThreshold) {
						double setpoint = (mVisionSetpoints[2] + mVisionSetpoints[1]) / 2;
						System.out.println("Chosen one: " + setpoint);
						commands.robotSetpoints.sliderCustomSetpoint = Optional.of(setpoint);
						break;
					}
					// good value on right side but out of bounds on left side
					else if (mVisionSetpoints[0] <= -7 && mVisionSetpoints[2] < 7 && mVisionSetpoints[2] > -7) {
						double setpoint = -7;
						System.out.println("Chosen one: " + setpoint);
						commands.robotSetpoints.sliderCustomSetpoint = Optional.of(setpoint);
						break;
					}
					// good value on left side but out of bounds on right side
					else if (mVisionSetpoints[2] >= 7 && mVisionSetpoints[0] < 7 && mVisionSetpoints[0] > -7) {
						double setpoint = 7;
						System.out.println("Chosen one: " + setpoint);
						commands.robotSetpoints.sliderCustomSetpoint = Optional.of(setpoint);
						break;
					}
				}
			}
		try {
			System.out.println(commands.robotSetpoints.sliderCustomSetpoint.get());
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
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
	 * @return Whether a scoring setpoint has been determined
	 * 			and the slider is on target
	 */
	@Override
	public boolean finished() {
		return mState==SamplingState.SCORE &&
				Robot.getRobotState().sliderVelocity==0 &&
				System.currentTimeMillis() - mStartTime > 200;
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
		return "MultiSampleVisionSliderRoutine";
	}
}

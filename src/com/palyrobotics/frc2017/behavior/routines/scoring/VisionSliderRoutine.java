package com.palyrobotics.frc2017.behavior.routines.scoring;

import java.util.Optional;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.subsystems.Slider;
import com.palyrobotics.frc2017.subsystems.Spatula;
import com.palyrobotics.frc2017.subsystems.Slider.SliderState;
import com.palyrobotics.frc2017.subsystems.Slider.SliderTarget;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.vision.AndroidConnectionHelper;

/**
 * Routine that uses vision to move slider
 * @author Nihar Mitra
 */
public class VisionSliderRoutine extends Routine {
	private double mStartTime = 0;
	
	// Used to make sure vision setpoint is only sent once
	private enum VisionPositioningState {
		START, SENT
	}
	private VisionPositioningState mState = VisionPositioningState.START;
	
	private final double kOffset = 7.5;	// Compensation for camera position
	
	public VisionSliderRoutine() {
	}
	
	/**
	 * Register start time
	 */
	@Override
	public void start() {
		mStartTime = System.currentTimeMillis();
	}

	/**
	 * Determine vision setpoint and update slider setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		commands.robotSetpoints.sliderSetpoint = SliderTarget.CUSTOM;
		commands.wantedSpatulaState = Spatula.SpatulaState.UP;
		double visionSetpoint = AndroidConnectionHelper.getInstance().getXDist();
		// out of range of motion, probably false positive, might be on left side
		if (visionSetpoint >= 1.5) {
			visionSetpoint = -7;
		} 
		else if (visionSetpoint <= -7-kOffset) {
			visionSetpoint = -7;
		} // extend motion
		else if (visionSetpoint < 0) {
			visionSetpoint += kOffset;
		} else {
			visionSetpoint += kOffset;
		}
		System.out.println("Vision setpoint pre min/max: "+visionSetpoint);
		visionSetpoint = Math.max(-7, Math.min(visionSetpoint, 7));
		if (commands.robotSetpoints.sliderCustomSetpoint.isPresent()) {
			System.out.println("Vision setpoint: "+visionSetpoint);
		}
		commands.robotSetpoints.sliderCustomSetpoint =
				Optional.of(visionSetpoint * Constants.kSliderRevolutionsPerInch);
		
		switch(mState) {
		case START:	// run only once
			commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
			try {
				slider.run(commands, this);
			} catch (IllegalAccessException e) {
				e.printStackTrace();
			}
			mState = VisionPositioningState.SENT;
			break;
		case SENT:
			commands.wantedSliderState = Slider.SliderState.CUSTOM_POSITIONING;
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
		commands.wantedSliderState = SliderState.IDLE;
		commands.robotSetpoints.sliderCustomSetpoint = Optional.empty();
		try {
			slider.run(commands, this);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
		return commands;
	}

	/**
	 * @return Whether setpoint has been sent and slider is on target
	 */
	@Override
	public boolean finished() {
		return mState==VisionPositioningState.SENT && 
				(System.currentTimeMillis() - mStartTime > 200) &&
				Robot.getRobotState().sliderVelocity == 0;
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
		return "SliderVisionPositioningRoutine";
	}
}
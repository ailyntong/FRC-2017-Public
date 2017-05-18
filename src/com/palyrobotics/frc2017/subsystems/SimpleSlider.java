package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.Commands.JoystickInput;
import com.palyrobotics.frc2017.robot.HardwareAdapter;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.subsystems.Slider.SliderTarget;
import com.palyrobotics.frc2017.util.CANTalonOutput;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

public class SimpleSlider extends Subsystem implements SubsystemLoop {
	private static SimpleSlider instance = new SimpleSlider();
	public static SimpleSlider getInstance() {
		return instance;
	}
	private RobotState mRobotState;
	private CANTalonOutput mOutput = new CANTalonOutput();
	
	public enum SliderDirection {
		LEFT,
		RIGHT,
		IDLE
	}
	private SliderDirection mSliderDirection;
	public SimpleSlider() {
		super("Sipmle Slider");
	}

	@Override
	public void start() {
		mSliderDirection = SliderDirection.IDLE;
		mOutput.setPercentVBus(0);
	}

	@Override
	public void stop() {
		mSliderDirection = SliderDirection.IDLE;
		mOutput.setPercentVBus(0);
	}

	@Override
	public String getStatus() {
		return "";
	}

	@Override 
	public void update(Commands commands, RobotState robotState) {
		mSliderDirection = commands.wantedSimpleSliderState;
		mRobotState = robotState;
		switch(mSliderDirection) {
		case LEFT:
			mOutput.setPercentVBus(-.25);
			System.out.println("Moving left");
			break;
		case RIGHT:
			mOutput.setPercentVBus(.25);
			System.out.println("Moving Right");
			break;
		case IDLE:
			mOutput.setPercentVBus(0);
			break;
		}
	}
	public CANTalonOutput getOutput() {
		return mOutput;
	}

	public SliderDirection getSliderState() {
		return mSliderDirection;
	}

}

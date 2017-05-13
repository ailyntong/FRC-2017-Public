package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

public class SimpleIntake extends Subsystem implements SubsystemLoop {
	private static SimpleIntake instance = new SimpleIntake();
	public SimpleIntake() {
		super("Simple Intake");
	}
	
	private enum IntakeState {
		IDLE,
		INTAKING,
		EXPELLING
	}

	private IntakeState mIntakeState;
	
	private double mOutput;
	
	private RobotState mRobotState;
	
	@Override
	public void start() {
		mIntakeState = IntakeState.IDLE;
	}

	@Override
	public void stop() {
		mIntakeState = IntakeState.IDLE;
	}

	@Override
	public String getStatus() {
		return null;
	}
	
	public double getOutput() {
		return mOutput;
	}
	@Override
	public void update(Commands commands, RobotState robotState) {
		mRobotState = robotState;
		switch(mIntakeState) {
		case IDLE:
			mOutput = 0;
			break;
		case INTAKING:
			mOutput = 1;
			break;
		case EXPELLING:
			mOutput = -1;
			break;
		}
	}
	
}

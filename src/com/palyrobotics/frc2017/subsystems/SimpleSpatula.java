package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class SimpleSpatula extends Subsystem implements SubsystemLoop{
	private static SimpleSpatula instance = new SimpleSpatula();
	public static SimpleSpatula getInstance() {
		return instance;
	}
	private RobotState mRobotState;
	private DoubleSolenoid.Value mOutput = DoubleSolenoid.Value.kOff;
	public SimpleSpatula() {
		super("Simple Spatula");
	}
	public enum SimpleSpatulaState {UP, DOWN}
	private SimpleSpatulaState mSimpleSpatulaState;
	@Override
	public void start() {
		mSimpleSpatulaState = SimpleSpatulaState.UP;
		mOutput = DoubleSolenoid.Value.kOff;
	}

	@Override
	public void stop() {
		mSimpleSpatulaState = SimpleSpatulaState.UP;
		mOutput = DoubleSolenoid.Value.kOff;

	}

	@Override
	public String getStatus() {
		return "";
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		switch(commands.wantedSimpleSpatulaState) {
		case UP:
			mOutput = DoubleSolenoid.Value.kReverse;
			mSimpleSpatulaState = SimpleSpatulaState.UP;
			break;
		case DOWN:
			mOutput = DoubleSolenoid.Value.kForward;
			mSimpleSpatulaState = SimpleSpatulaState.DOWN;
			break;
		}

	}
	
	public DoubleSolenoid.Value getOutput() {
		return mOutput;
	}
}

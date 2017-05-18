package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.util.CANTalonOutput;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

/**
 * STEIK CLIMBER
 * @author Jason
 * Consists of a motor that winches up the robot to climb at the end of the match.
 * The winch/climber is controlled by a  CANSD540
 */
public class SimpleClimber extends Subsystem implements SubsystemLoop {
	private static SimpleClimber instance = new SimpleClimber();
	private RobotState mRobotState;
	public static SimpleClimber getInstance() {
		return instance;
	}
	
	//Speed at which the robot is currently climbing, changes if ClimberState is CLIMBING
	private CANTalonOutput mOutput = new CANTalonOutput();
;

	public SimpleClimber() {
		super("Simple Climber");
	}
	
	public enum ClimberState{ CLIMBING, IDLE }
	private ClimberState climberState;
	
	@Override
	public void start() {
		mOutput.setPercentVBus(0);
	}

	@Override
	public void stop() {
		mOutput.setPercentVBus(0);
	}
	/**
	 * Updates climber subsystem, and sets the climberSpeed to wantedClimberSpeed
	 */
	@Override
	public void update(Commands commands, RobotState robotState) {
		climberState = commands.wantedSimpleClimberState;
		switch(climberState){
		case CLIMBING:
			mOutput.setPercentVBus(0.3);
			break;
		case IDLE:
			mOutput.setPercentVBus(0);
			break;
		}
	}
	/**
	 * @return climberSpeed
	 */
	public CANTalonOutput getOutput() {
		return mOutput;
	}

	@Override
	public String getStatus() {
		return "";
	}
	public ClimberState getClimberState(){
		return climberState;
	}
}
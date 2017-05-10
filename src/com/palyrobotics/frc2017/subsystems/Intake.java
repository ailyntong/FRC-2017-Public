package com.palyrobotics.frc2017.subsystems;

import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.util.Subsystem;
import com.palyrobotics.frc2017.util.archive.SubsystemLoop;

/**
 * Represents the intake
 * Single motor that spins at set speeds
 * @author Ailyn Tong
 *
 */
public class Intake extends Subsystem implements SubsystemLoop {
	// Singleton setup
	private static Intake instance = new Intake();
	public static Intake getInstance() {
		return instance;
	}
	
	// Determines intake behavior
	public enum IntakeState { IDLE, INTAKE, EXPEL }
	
	private double mOutput;
	
	/**
	 * Constructor
	 */
	private Intake() {
		super("Intake");
	}

	@Override
	public void start() {
	}

	@Override
	public void stop() {
	}

	@Override
	public void update(Commands commands, RobotState robotState) {
		switch (commands.wantedIntakeState) {
		// Sets output to a constant speed
		case IDLE:
			mOutput = 0;
			break;
		case INTAKE:
			mOutput = Constants.kManualIntakeSpeed;
			break;
		case EXPEL:
			mOutput = Constants.kManualExhaustSpeed;
			break;
		}
	}

	/**
	 * @return The current intake output
	 */
	public double getOutput() {
		return mOutput;
	}

	@Override
	public String getStatus() {
		return "";
	}

}

package com.palyrobotics.frc2017.behavior.routines.drive;

import com.ctre.CANTalon;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.subsystems.controllers.CANTalonDriveController;
import com.palyrobotics.frc2017.util.archive.DriveSignal;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Created by Nihar on 2/12/17.
 * @author Nihar
 * Should be used to set the drivetrain to an offboard closed loop cantalon
 */
public class CANTalonRoutine extends Routine {
	private boolean mRelativeSetpoint = false;	// Whether the setpoint is relative or absolute
	private final DriveSignal mSignal;	// Desired output
	
	private double mTimeout;	// Time limit in milliseconds
	private double mStartTime;	// Start time in milliseconds
	
	/**
	 * Constructor
	 * @param controller DriveSignal with desired CANTalonOutputs
	 * @param relativeSetpoint Whether the setpoint is relative or absolute
	 */
	public CANTalonRoutine(DriveSignal controller, boolean relativeSetpoint) {
		this.mSignal = controller;
		this.mTimeout = 1 << 30;
		this.mRelativeSetpoint = relativeSetpoint;
	}

	/*
	  * Setpoint is relative when you want it to be updated on start
	  * For position and motion magic only
	  * @param controller DriveSignal with desired CANTalonOutputs
	  * @param relativeSetpoint Whether the setpoint is relative or absolute
	  * @param timeout Time limit in seconds
	  */
	public CANTalonRoutine(DriveSignal controller, boolean relativeSetpoint, double timeout) {
		this.mSignal = controller;
		this.mRelativeSetpoint = relativeSetpoint;
		this.mTimeout = timeout * 1000;
	}

	/**
	 * Initialize start time and talon setpoints
	 * Set drive controller
	 */
	@Override
	public void start() {
		
		mStartTime = System.currentTimeMillis();
		
		if (mRelativeSetpoint) {
			if (mSignal.leftMotor.getControlMode() == CANTalon.TalonControlMode.MotionMagic) {
				mSignal.leftMotor.setMotionMagic(mSignal.leftMotor.getSetpoint()+
								Robot.getRobotState().drivePose.leftEnc,
						mSignal.leftMotor.gains,
						mSignal.leftMotor.cruiseVel, mSignal.leftMotor.accel);
				mSignal.rightMotor.setMotionMagic(mSignal.rightMotor.getSetpoint()+
								Robot.getRobotState().drivePose.rightEnc,
						mSignal.rightMotor.gains,
						mSignal.rightMotor.cruiseVel, mSignal.rightMotor.accel);
			}
			else if (mSignal.leftMotor.getControlMode() == CANTalon.TalonControlMode.Position) {
				mSignal.leftMotor.setPosition(mSignal.leftMotor.getSetpoint()+
						Robot.getRobotState().drivePose.leftEnc, mSignal.leftMotor.gains);
				mSignal.rightMotor.setPosition(mSignal.rightMotor.getSetpoint()+
						Robot.getRobotState().drivePose.rightEnc, mSignal.rightMotor.gains);
				

			}
		}
		drive.setCANTalonController(mSignal);
		System.out.println("Sent drivetrain signal "+mSignal.toString());
	}

	/**
	 * Update setpoints
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		Commands output = commands.copy();
		output.wantedDriveState = Drive.DriveState.OFF_BOARD_CONTROLLER;
		return output;
	}

	/**
	 * Stop drivetrain
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		drive.setNeutral();
		commands.wantedDriveState = Drive.DriveState.NEUTRAL;
		return commands;
	}

	/**
	 * @return True if the controller has finished or the time limit has passsed,
	 * 		or if there is no drive controller
	 */
	@Override
	public boolean finished() {
		// Wait for controller to be added before finishing routine
		if (mSignal.leftMotor.getSetpoint() != Robot.getRobotState().leftSetpoint) {
			System.out.println("Mismatched desired talon and actual talon setpoints! desired, actual");
			System.out.println("Left: " + mSignal.leftMotor.getSetpoint()+", "+Robot.getRobotState().leftSetpoint);
			return false;
		}
		else if (mSignal.rightMotor.getSetpoint() != Robot.getRobotState().rightSetpoint) {
			System.out.println("Mismatched desired talon and actual talon setpoints! desired, actual");
			System.out.println("Right: " + mSignal.rightMotor.getSetpoint()+", "+Robot.getRobotState().rightSetpoint);
			return false;
		}
		else if (mSignal.leftMotor.getControlMode() != Robot.getRobotState().leftControlMode) {
			System.out.println("Mismatched desired talon and actual talon states!");
			System.out.println(mSignal.leftMotor.getControlMode() + ", "+Robot.getRobotState().leftControlMode);
			return false;
		}
		else if (mSignal.rightMotor.getControlMode() != Robot.getRobotState().rightControlMode) {
			System.out.println("Mismatched desired talon and actual talon states!");
			System.out.println(mSignal.rightMotor.getControlMode()+","+Robot.getRobotState().rightControlMode);
			return false;
		}
		if (!drive.hasController() || (drive.getController().getClass() == CANTalonDriveController.class && drive.controllerOnTarget())) {
		}
		return !drive.hasController() || System.currentTimeMillis() > this.mTimeout+mStartTime || (drive.getController().getClass() == CANTalonDriveController.class && drive.controllerOnTarget());
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{Drive.getInstance()};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		return "DriveCANTalonRoutine";
	}
}

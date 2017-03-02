package com.palyrobotics.frc2017.auto.modes;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.CANTalonRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.EncoderTurnAngleRoutine;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Constants2016;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.util.archive.DriveSignal;

import java.util.ArrayList;

/**
 * Created by Nihar on 2/11/17.
 * BBTurnAngle might be replaced with EncoderTurnAngle if no gyro
 */
public class CenterPegAutoMode extends AutoModeBase {
	// Represents the variation of center peg auto based on what to do after scoring
	public enum CenterAutoVariant {
		NOTHING, CROSS_LEFT, CROSS_RIGHT
	}
	private final CenterAutoVariant mVariant;
	private SequentialRoutine mSequentialRoutine;

	private Gains mGains;

	public CenterPegAutoMode(CenterAutoVariant direction) {
		mVariant = direction;
		if(Constants.kRobotName == Constants.RobotName.DERICA) {
			mGains = Gains.dericaPosition;
		} else {
			mGains = (Constants.kRobotName == Constants.RobotName.STEIK) ? Gains.steikPosition : Gains.aegirPosition;
		}
	}

	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}

	@Override
	public void prestart() {
		String log = "Starting Center Peg Auto Mode";
		// Construct sequence of routines to run
		ArrayList<Routine> sequence = new ArrayList<Routine>();
		// Straight drive distance to the center peg
		DriveSignal driveForward = DriveSignal.getNeutralSignal();
		double driveForwardSetpoint = Constants.kCenterPegDistanceInches * 
				((Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kDericaInchesToTicks
						: Constants.kDriveInchesToTicks );
		driveForward.leftMotor.setMotionMagic(driveForwardSetpoint+Robot.getRobotState().drivePose.leftEnc, mGains, 
			Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);
		driveForward.rightMotor.setMotionMagic(driveForwardSetpoint+Robot.getRobotState().drivePose.rightEnc, mGains, 
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);
		
		sequence.add(new CANTalonRoutine(driveForward));
		sequence.add(new TimeoutRoutine(2.5));	// Wait 2.5s so pilot can pull gear out

		// Back off from the peg after 2.5 seconds
		DriveSignal driveBack = DriveSignal.getNeutralSignal();
		double driveBackSetpoint = -25 * 
				((Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kDericaInchesToTicks
				: Constants.kDriveInchesToTicks);
		driveBack.leftMotor.setMotionMagic(driveBackSetpoint+Robot.getRobotState().drivePose.leftEnc, mGains,
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);
		driveBack.rightMotor.setMotionMagic(driveBackSetpoint+Robot.getRobotState().drivePose.rightEnc, mGains,
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);

		// If variant includes a cross, drive past the airship after turn angle
		DriveSignal passAirship = DriveSignal.getNeutralSignal();
		double passAirshipSetpoint = 50 * 
				((Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kDericaInchesToTicks
				: Constants.kDriveInchesToTicks);
		passAirship.leftMotor.setMotionMagic(passAirshipSetpoint+Robot.getRobotState().drivePose.leftEnc, mGains,
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);
		passAirship.rightMotor.setMotionMagic(passAirshipSetpoint+Robot.getRobotState().drivePose.rightEnc, mGains,
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);

		DriveSignal crossOver = DriveSignal.getNeutralSignal();
		double crossOverSetpoint = 20 * 
				((Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kDericaInchesToTicks
				: Constants.kDriveInchesToTicks);
		crossOver.leftMotor.setMotionMagic(crossOverSetpoint+Robot.getRobotState().drivePose.leftEnc, mGains,
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);
		crossOver.rightMotor.setMotionMagic(crossOverSetpoint+Robot.getRobotState().drivePose.rightEnc, mGains,
				Gains.kAegirDriveMotionMagicCruiseVelocity, Gains.kAegirDriveMotionMagicMaxAcceleration);

		switch (mVariant) {
			case NOTHING:
				break;
			case CROSS_LEFT:
				sequence.add(new CANTalonRoutine(driveBack));
				sequence.add(new EncoderTurnAngleRoutine(-90));
				sequence.add(new CANTalonRoutine(passAirship));
				sequence.add(new EncoderTurnAngleRoutine(90));
				sequence.add(new CANTalonRoutine(crossOver));
				log += " and crossing left";
				break;
			case CROSS_RIGHT:
				sequence.add(new CANTalonRoutine(driveBack));
				sequence.add(new EncoderTurnAngleRoutine(90));
				sequence.add(new CANTalonRoutine(passAirship));
				sequence.add(new EncoderTurnAngleRoutine(-90));
				sequence.add(new CANTalonRoutine(crossOver));
				log += " and crossing right";
				break;
		}

		mSequentialRoutine = new SequentialRoutine(sequence);
		System.out.println(log);
	}
	@Override
	public String toString() {
		String name = "CenterPeg";
		switch (mVariant) {
			case NOTHING:
				break;
			case CROSS_LEFT:
				name += "_CrossLeft";
				break;
			case CROSS_RIGHT:
				name += "_CrossRight";
				break;
		}
		return name;
	}

}

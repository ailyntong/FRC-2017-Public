package com.palyrobotics.frc2017.auto.modes.archive;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.routines.drive.CANTalonRoutine;
import com.palyrobotics.frc2017.config.AutoDistances;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Constants2016;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.util.archive.DriveSignal;

/**
 * Motion magic baseline autonomous
 * @author Nihar Mitra
 */
public class BaseLineAutoMode extends AutoModeBase {
	private CANTalonRoutine mRoutine;
	private final CenterPegAutoMode.Alliance mAlliance;	// Red vs blue
	private Gains mGains;	// Motion magic gains

	/**
	 * Constructor
	 * @param alliance Red or blue side
	 */
	public BaseLineAutoMode(CenterPegAutoMode.Alliance alliance) {
		mAlliance = alliance;
		// Initialize gains for appropriate robot
		if(Constants.kRobotName == Constants.RobotName.DERICA) {
			mGains = Gains.dericaPosition;
		} else {
			mGains = Gains.steikLongDriveMotionMagicGains;
		}
	}

	@Override
	public Routine getRoutine() {
		// Drive straight until baseline
		DriveSignal driveForward = DriveSignal.getNeutralSignal();
		double setpoint =
				((mAlliance == CenterPegAutoMode.Alliance.BLUE) ? AutoDistances.kBlueBaseLineDistanceInches : AutoDistances.kRedBaseLineDistanceInches)
						*
				((Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kDericaInchesToTicks
						: Constants.kDriveTicksPerInch);
		driveForward.leftMotor.setMotionMagic(setpoint, mGains,
			Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		driveForward.rightMotor.setMotionMagic(setpoint, mGains,
				Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		mRoutine = new CANTalonRoutine(driveForward, true);
		return mRoutine;
	}

	@Override
	public void prestart() {
		System.out.println("Starting Base Line Auto Mode");
	}

	@Override
	public String toString() {
		return (mAlliance == CenterPegAutoMode.Alliance.BLUE) ? "BlueBaseLine" : "RedBaseLine";
	}
}

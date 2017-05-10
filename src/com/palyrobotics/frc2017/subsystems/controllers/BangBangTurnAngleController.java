package com.palyrobotics.frc2017.subsystems.controllers;

import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Constants2016;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.Pose;
import com.palyrobotics.frc2017.util.archive.DriveSignal;

/**
 * Turns drivetrain using the gyroscope and bang-bang control loop
 * @author Robbie, Nihar
 *
 */
public class BangBangTurnAngleController implements Drive.DriveController {
	
	private double mPower;	// constant output
	private double mTargetHeading;	// Relative setpoint in degrees
	private Pose mCachedPose;	// Store previous drive pose

	/**
	 * Constructor
	 * @param currentPose Pass in the latest robot state
	 * @param heading Degrees relative to current state to turn
	 */
	public BangBangTurnAngleController(Pose currentPose, double heading) {
		this.mPower = (Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kTurnAngleSpeed : Constants.kTurnInPlacePower;
		this.mCachedPose = currentPose;
		this.mTargetHeading = this.mCachedPose.heading + heading;
		System.out.println("Starting Heading " + this.mCachedPose.heading);
		System.out.println("Target heading: "+this.mTargetHeading);
	}
	
	/**
	 * Updates output based on relative error to target heading
	 */
	@Override
	public DriveSignal update(RobotState state) {
		if (this.onTarget()) {
			return DriveSignal.getNeutralSignal();
		}
		mCachedPose = state.drivePose;
//		System.out.println("Current Pose: " + mCachedPose.heading);
		DriveSignal output = DriveSignal.getNeutralSignal();
		if (mCachedPose.heading < mTargetHeading) {
			output.leftMotor.setPercentVBus(this.mPower);
			output.rightMotor.setPercentVBus(-(this.mPower));
		} else {
			output.leftMotor.setPercentVBus(-(this.mPower));
			output.rightMotor.setPercentVBus(this.mPower);
		}
		return output;
	}

	/**
	 * @return Setpoint
	 */
	@Override
	public Pose getSetpoint() {
		mCachedPose.heading = mTargetHeading;
		Pose setpoint = new Pose(0,0,0,0,0,0,0,0);
		return mCachedPose;
	}

	/**
	 * @return Whether heading is within acceptable tolerance
	 */
	@Override
	public boolean onTarget() {
		double tolerance = (Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kAcceptableGyroTurnError : Constants.kAcceptableTurnAngleError;
		return Math.abs(mCachedPose.heading - mTargetHeading) < tolerance;
	}

}

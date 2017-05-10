package com.palyrobotics.frc2017.subsystems.controllers;

import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Constants2016;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.subsystems.Drive.DriveController;
import com.palyrobotics.frc2017.util.CANTalonOutput;
import com.palyrobotics.frc2017.util.Pose;
import com.palyrobotics.frc2017.util.archive.DriveSignal;

/**
 * Controller that uses a gyroscope to update a motion magic control loop
 * @author Ailyn Tong
 */
public class GyroMotionMagicTurnAngleController implements DriveController {
	private Pose mCachedPose;	// Store previous pose
	private final double mTargetHeading;	// Absolute setpoint in degrees
	private double mLeftTarget, mRightTarget;	// Absolute setpoint in encoder ticks
	private CANTalonOutput mLeftOutput, mRightOutput;	// Master talon outputs
	
	private final Gains mGains;	// Motion magic gains
	private final double mCruiseVel, mMaxAccel;	// Motion magic gains
	
	private final double kInchesPerDegree, kTicksPerInch;	// Unit conversion
	private final double kTolerance;	// Acceptable angle tolerance in degrees
		
	/**
	 * Constructor
	 * @param priorSetpoint Original pose
	 * @param angle Relative setpoint in degrees
	 */
	public GyroMotionMagicTurnAngleController(Pose priorSetpoint, double angle) {
		mCachedPose = priorSetpoint;
		mTargetHeading = priorSetpoint.heading + angle;
		
		// Instantiate constants for correct robot
		if (Constants.kRobotName == Constants.RobotName.DERICA) {
			mGains = Gains.dericaPosition;
			mCruiseVel = Gains.kDericaTurnMotionMagicCruiseVelocity;
			mMaxAccel = Gains.kDericaTurnMotionMagicCruiseAccel;
			kInchesPerDegree = 1 / Constants2016.kDericaInchesToDegrees;
			kTicksPerInch = Constants2016.kDericaInchesToTicks;
			kTolerance = Constants2016.kAcceptableGyroTurnError;
		} else {
			mGains = Gains.steikTurnMotionMagicGains;
			mCruiseVel = Gains.kSteikTurnMotionMagicCruiseVelocity;
			mMaxAccel = Gains.kSteikTurnMotionMagicMaxAcceleration;
			kInchesPerDegree = Constants.kDriveInchesPerDegree;
			kTicksPerInch = Constants.kDriveTicksPerInch;
			kTolerance = Constants.kAcceptableTurnAngleError;
		}
		// Initialize setpoints
		System.out.println("Current heading: " + mCachedPose.heading);
		System.out.println("Target heading: " + mTargetHeading);
		mLeftTarget = priorSetpoint.leftEnc - (angle * kInchesPerDegree * kTicksPerInch);
		mRightTarget = priorSetpoint.rightEnc + (angle * kInchesPerDegree * kTicksPerInch);
		// Initialize CANTalonOutputs
		mLeftOutput = new CANTalonOutput();
		mLeftOutput.setMotionMagic(mLeftTarget, mGains, mCruiseVel, mMaxAccel);
		mRightOutput = new CANTalonOutput();
		mRightOutput.setMotionMagic(mRightTarget, mGains, mCruiseVel, mMaxAccel);
	}

	/**
	 * Uses error to update motion magic setpoint
	 * @return Updated DriveSignal
	 */
	@Override
	public DriveSignal update(RobotState state) {
		mCachedPose = state.drivePose;
		double error = mTargetHeading - mCachedPose.heading;
//		System.out.println(mCachedPose.headingVelocity);
		// Compensate for current motion
//		error -= mCachedPose.headingVelocity*Constants.kSubsystemLooperDt;
		mLeftTarget = mCachedPose.leftEnc - (error * kInchesPerDegree * kTicksPerInch);
		mRightTarget = mCachedPose.rightEnc + (error * kInchesPerDegree * kTicksPerInch);
		mLeftOutput.setMotionMagic(mLeftTarget, mGains, mCruiseVel, mMaxAccel);
		mRightOutput.setMotionMagic(mRightTarget, mGains, mCruiseVel, mMaxAccel);

		return new DriveSignal(mLeftOutput, mRightOutput);
	}

	/**
	 * @return Setpoint
	 */
	@Override
	public Pose getSetpoint() {
		return new Pose(0, 0, 0, 0, 0, 0, mTargetHeading, 0);
	}

	/**
	 * @return Whether or not the robot is within position and velocity tolerance
	 */
	@Override
	public boolean onTarget() {
		// Wait for controller to be added before finishing routine
		if (mCachedPose == null) {
			System.out.println("Cached pose is null");
			return false;
		}
		System.out.println("On target "+(Math.abs(Robot.getRobotState().drivePose.heading - mTargetHeading) < 3.4));
		System.out.println(Robot.getRobotState().drivePose.heading);
		return Math.abs(Robot.getRobotState().drivePose.heading - mTargetHeading) < kTolerance
				&& Math.abs(Robot.getRobotState().drivePose.headingVelocity)<0.05;
	}

}

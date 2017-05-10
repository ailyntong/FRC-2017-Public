package com.palyrobotics.frc2017.subsystems.controllers;

import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.config.Constants.RobotName;
import com.palyrobotics.frc2017.config.Constants2016;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.robot.team254.lib.util.SynchronousPID;
import com.palyrobotics.frc2017.subsystems.Drive.DriveController;
import com.palyrobotics.frc2017.util.CANTalonOutput;
import com.palyrobotics.frc2017.util.Pose;
import com.palyrobotics.frc2017.util.archive.DriveSignal;

/**
 * Onboard drive straight control loop using SynchronousPID
 * @author Robbie Selwyn
 */
public class DriveStraightController implements DriveController {

	private Pose mCachedPose;	// Store previous drive pose
	private double mTarget;	// Distance in inches
	private Gains mGains;	// PID gains
	
	private SynchronousPID mForwardPID;	// Distance loop
	private SynchronousPID mHeadingPID;	// Heading loop
	
	private final double kTolerance;	// Acceptable distance tolerance in inches
	
	/**
	 * Constructor
	 * @param priorSetpoint Original drivetrain pose
	 * @param distance Relative target in inches
	 */
	public DriveStraightController(Pose priorSetpoint, double distance) {
		mTarget = (priorSetpoint.leftEnc + priorSetpoint.rightEnc)/2 + (distance * Constants.kDriveTicksPerInch);
		System.out.println("Target: "+mTarget);
		mCachedPose = priorSetpoint;
		
		// Initialize PID
		mGains = new Gains(.00035, 0.000004, 0.002, 0, 200, 0);
		kTolerance = (Constants.kRobotName == RobotName.DERICA) ? Constants2016.kAcceptableDriveError : Constants.kAcceptableDrivePositionError;
		mForwardPID = new SynchronousPID(mGains.P, mGains.I, mGains.D, mGains.izone);
		mHeadingPID = new SynchronousPID(Gains.kSteikDriveStraightTurnkP, 0, 0.005);
		mForwardPID.setOutputRange(-1, 1);
		mHeadingPID.setOutputRange(-0.2, 0.2);
		mForwardPID.setSetpoint(mTarget);
		mHeadingPID.setSetpoint(priorSetpoint.heading);
		
	}

	/**
	 * @return Whether the robot is within position and velocity tolerance of target
	 */
	@Override
	public boolean onTarget() {
		if (mCachedPose == null) {
			System.out.println("Cached pose is null");
			return false;
		}
		
		return Math.abs(Robot.getRobotState().drivePose.heading) < kTolerance &&
				Math.abs((Robot.getRobotState().drivePose.leftEnc + Robot.getRobotState().drivePose.rightEnc)/2  - mTarget) < kTolerance
				&& Math.abs(Robot.getRobotState().drivePose.leftSpeed)<0.05
				&& Math.abs(Robot.getRobotState().drivePose.rightSpeed)<0.05;
	}
	
	/**
	 * Calculates and updates output using SynchronousPID
	 * @return New drivetrain output
	 */
	@Override
	public DriveSignal update(RobotState state) {
		CANTalonOutput leftOutput = new CANTalonOutput();
		CANTalonOutput rightOutput = new CANTalonOutput();
		mCachedPose = state.drivePose;
		// Take average of left and right distances
		double distanceSoFar = state.drivePose.leftEnc+state.drivePose.rightEnc;
		distanceSoFar /= 2;
		double throttle = mForwardPID.calculate(distanceSoFar);
//		double turn = headingPID.calculate(state.drivePose.heading) * Constants.kDriveInchesPerDegree;
		double turn = 0;
		leftOutput.setPercentVBus(throttle + turn);
		rightOutput.setPercentVBus(throttle - turn);
		
		System.out.println(mForwardPID.getError());

		return new DriveSignal(leftOutput, rightOutput);
	}

	/**
	 * @return Setpoint
	 */
	@Override
	public Pose getSetpoint() {
		return new Pose(mTarget, 0, 0, mTarget, 0, 0, 0, 0, 0, 0);
	}

}

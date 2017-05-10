package com.palyrobotics.frc2017.subsystems.controllers;

import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.config.RobotState;
import com.palyrobotics.frc2017.config.dashboard.DashboardManager;
import com.palyrobotics.frc2017.robot.team254.lib.util.ChezyMath;
import com.palyrobotics.frc2017.robot.team254.lib.util.SynchronousPID;
import com.palyrobotics.frc2017.subsystems.Drive;
import com.palyrobotics.frc2017.util.Pose;
import com.palyrobotics.frc2017.util.archive.DriveSignal;
import com.palyrobotics.frc2017.util.archive.team254.trajectory.LegacyTrajectoryFollower;
import com.palyrobotics.frc2017.util.logger.Logger;
import com.team254.lib.trajectory.Path;

/**
 * Created by Nihar on 4/5/17.
 * Controller that follows a trajectory (aka motion profile)
 */
public class TrajectoryFollowingController implements Drive.DriveController {
	// Trajctory followers for each side of the robot
	private LegacyTrajectoryFollower mLeftFollower = new LegacyTrajectoryFollower("left");
	private LegacyTrajectoryFollower mRightFollower = new LegacyTrajectoryFollower("right");

	private boolean mGyroCorrection;	// Whether to use gyro to correct heading
	private boolean mIllegalPath;	// Whether the path is null

	private SynchronousPID mHeadingPID;	// gyro correction

	/**
	 * Constructor
	 * @param path Trajectory to follow
	 * @param gains Trajectory PID gains
	 * @param correctUsingGyro Whether to use gyro for heading correction
	 * @param inverted Whether the path should be left/right inverted
	 */
	public TrajectoryFollowingController(Path path, Gains.TrajectoryGains gains, boolean correctUsingGyro, boolean inverted) {
		// Initialize PID
		mHeadingPID = new SynchronousPID(gains.turnP, 0, gains.turnD);
		mHeadingPID.setOutputRange(-0.15, 0.15);
		mHeadingPID.setSetpoint(0);

		// set trajectory gains
		mLeftFollower.configure(gains.P, 0, gains.D,
				gains.V, gains.A);
		mRightFollower.configure(gains.P, 0, gains.D,
				gains.V, gains.A);

		// set goals and paths
		if (path == null) {
			mIllegalPath = true;
			System.out.println("No path!");
			return;
		} else {
			mIllegalPath = false;
		}
		// Set trajectory
		if (inverted) {
			path.getRightWheelTrajectory().setInvertedY(inverted);
			path.getLeftWheelTrajectory().setInvertedY(inverted);
		}
		mRightFollower.setTrajectory(path.getRightWheelTrajectory());
		mLeftFollower.setTrajectory(path.getLeftWheelTrajectory());
		
		mGyroCorrection = correctUsingGyro;
	}

	/**
	 * Calculates output based on path
	 * @return Updated DriveSignal
	 */
	@Override
	public DriveSignal update(RobotState state) {
		if (mIllegalPath) {
			return DriveSignal.getNeutralSignal();
		}
		if (onTarget()) {
			return DriveSignal.getNeutralSignal();
		}
		DriveSignal driveSignal = DriveSignal.getNeutralSignal();

		double leftPower = mLeftFollower.calculate(state.drivePose.leftEnc/Constants.kDriveTicksPerInch/12);
		double rightPower = mRightFollower.calculate(state.drivePose.rightEnc/Constants.kDriveTicksPerInch/12);
		
		double gyroError = 0;

		if (!mGyroCorrection) {
			driveSignal.leftMotor.setPercentVBus(leftPower);
			driveSignal.rightMotor.setPercentVBus(rightPower);
		} else {
			gyroError = ChezyMath.getDifferenceInAngleRadians(Math.toRadians(state.drivePose.heading), mLeftFollower.getHeading());
			gyroError = Math.toDegrees(gyroError);
			double gyroCorrection = mHeadingPID.calculate(gyroError);
			System.out.println("Gyro correction: "+gyroCorrection);
			Logger.getInstance().logSubsystemThread(gyroCorrection);
			driveSignal.leftMotor.setVoltage((leftPower+gyroCorrection)*12);
			driveSignal.rightMotor.setVoltage((rightPower-gyroCorrection)*12);
		}
		
		DashboardManager.getInstance().updateCANTable(mLeftFollower.getCanTableString() + ", " + mRightFollower.getCanTableString() + ", " + gyroError);
		
		return driveSignal;
	}

	/**
	 * @return Setpoint
	 */
	@Override
	public Pose getSetpoint() {
		return new Pose(0,0,0,0,0,0,0,0);
	}

	/**
	 * @return Whether or not the trajectory has completed
	 */
	@Override
	public boolean onTarget() {
		return !mIllegalPath | mLeftFollower.isFinishedTrajectory() && mRightFollower.isFinishedTrajectory();
	}
}

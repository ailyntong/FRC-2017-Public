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
 * Controller used for offboard turn angle
 * @author Eric Liu
 */
public class EncoderTurnAngleController implements DriveController {

	private Pose mCachedPose;				// Store previous drive pose
	private double mLeftTarget;				// Absolute setpoint in encoder ticks
	private double mRightTarget;			// Absolute setpoint in encoder ticks
	private final double kMaxAccel;			// m/s^2
	private final double kMaxVel;			// m/s
	private Gains mGains;					// PID gains
	private CANTalonOutput mLeftOutput;		// Left master talon output
	private CANTalonOutput mRightOutput;	// Right master talon output
	
	/**
	 * Constructor
	 * @param priorSetpoint Original drivetrain pose
	 * @param angle Relative target in degrees
	 */
	public EncoderTurnAngleController(Pose priorSetpoint, double angle) {
		// Initialize setpoints
		mLeftTarget = priorSetpoint.leftEnc + (angle * Constants.kDriveInchesPerDegree * Constants.kDriveTicksPerInch);
		System.out.println("Left target: "+mLeftTarget);
		mRightTarget = priorSetpoint.rightEnc - (angle * Constants.kDriveInchesPerDegree * Constants.kDriveTicksPerInch);
		System.out.println("Right target: "+mRightTarget);
		mCachedPose = priorSetpoint;
		// Initialize max acceleration and velocity
		this.kMaxAccel = (Constants.kRobotName == Constants.RobotName.DERICA) ? Gains.kDericaTurnMotionMagicCruiseVelocity : (72 * Constants.kDriveSpeedUnitConversion);
		this.kMaxVel = (Constants.kRobotName == Constants.RobotName.DERICA) ?  Gains.kDericaTurnMotionMagicCruiseAccel : (36 * Constants.kDriveSpeedUnitConversion);
		// Initialize PID gains
		if(Constants.kRobotName.equals(Constants.RobotName.STEIK)) {
			mGains = new Gains(6.0, 0.01, 210, 2.0, 50, 0.0);
		} else {
			mGains = Gains.dericaPosition;
		}
		// Initialize CANTalonOutputs
		mLeftOutput = new CANTalonOutput();
		mLeftOutput.setMotionMagic(mLeftTarget, mGains, kMaxVel, kMaxAccel);
		mRightOutput = new CANTalonOutput();
		mRightOutput.setMotionMagic(mRightTarget, mGains, kMaxVel, kMaxAccel);
	}

	/**
	 * @return Whether the robot is within position and velocity tolerance of target
	 */
	@Override
	public boolean onTarget() {
		if(Robot.getRobotState().leftSetpoint != mLeftOutput.getSetpoint() || Robot.getRobotState().rightSetpoint != mRightOutput.getSetpoint() ||
				Robot.getRobotState().leftControlMode != mLeftOutput.getControlMode() || Robot.getRobotState().rightControlMode != mRightOutput.getControlMode()) {
			System.out.println("Mismatched desired talon and actual talon states!");
			return false;
		}

		double positionTolerance = ((Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kAcceptableDrivePositionError : Constants.kAcceptableTurnAngleError) * 
				Constants.kDriveInchesPerDegree * Constants.kDriveTicksPerInch;
		double velocityTolerance = (Constants.kRobotName == Constants.RobotName.DERICA) ? Constants2016.kAcceptableDriveVelocityError : Constants.kAcceptableDriveVelocityError;

		if(mCachedPose == null) {
			System.out.println("Cached pose is null");
			return false;
		}
//		System.out.println("Left: " + Math.abs(leftTarget - cachedPose.leftEnc) + 
//				"Right: " + Math.abs(rightTarget - cachedPose.rightEnc));
		if(Math.abs(mCachedPose.leftSpeed) < velocityTolerance && Math.abs(mCachedPose.rightSpeed) < velocityTolerance &&
				Math.abs(mLeftTarget - mCachedPose.leftEnc) < positionTolerance && Math.abs(mRightTarget - mCachedPose.rightEnc) < positionTolerance) {
			System.out.println("turn angle done");
			return true;
		}
		else return false;
	}

	/**
	 * Updates controller's robot state
	 * @return Target signal
	 */
	@Override
	public DriveSignal update(RobotState state) {
		mCachedPose = state.drivePose;
		return new DriveSignal(mLeftOutput, mRightOutput);
	}

	/**
	 * @return Setpoint
	 */
	@Override
	public Pose getSetpoint() {
		return new Pose(mLeftTarget, 0, 0, mRightTarget, 0, 0, 0, 0, 0, 0);
	}

}

package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.robot.Robot;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Turns a specified angle
 * Use bang-bang if gyro is working, otherwise use encoder turn angle
 * @author Nihar
 */
public class SafetyTurnAngleRoutine extends Routine {
	private double mTargetAngle;	// Relative setpoint in degrees
	private Routine mRoutine;	// Turn angle routine to use

	/**
	 * Constructor
	 * @param angle Angle in degrees relative to current heading
	 */
	public SafetyTurnAngleRoutine(double angle) {
		this.mTargetAngle = angle;
	}

	/**
	 * Set routine
	 */
	@Override
	public void start() {
		if(Robot.getRobotState().drivePose.heading == -0.0) {
			System.out.println("Gyro broken");
			mRoutine = new EncoderTurnAngleRoutine(mTargetAngle);
        } else {
        	System.out.println("Gyro working!");
            mRoutine = new BBTurnAngleRoutine(mTargetAngle);
        }
		mRoutine.start();
	}

	/**
	 * Update routine
	 * @return Modified commands
	 */
	@Override
	public Commands update(Commands commands) {
		System.out.println("angle: "+Robot.getRobotState().drivePose.heading);
		return mRoutine.update(commands);
	}

	/**
	 * Cancel routine
	 * @return Modified commands
	 */
	@Override
	public Commands cancel(Commands commands) {
		return mRoutine.cancel(commands);
	}

	/**
	 * @return Whether the routine is finished
	 */
	@Override
	public boolean finished() {
		return mRoutine.finished();
	}

	/**
	 * @return Set of subsystems required by routine
	 */
	@Override
	public Subsystem[] getRequiredSubsystems() {
		return new Subsystem[]{drive};
	}

	/**
	 * @return Name of routine
	 */
	@Override
	public String getName() {
		String name = "SafetyTurnAngleRoutine_";
		if (mRoutine instanceof BBTurnAngleRoutine) {
			name += "GyroTurnAngle";
		} else {
			name += "EncoderTurnAngle";
		}
		return name;
	}
}

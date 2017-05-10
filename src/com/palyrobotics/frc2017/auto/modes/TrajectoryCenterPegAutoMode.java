package com.palyrobotics.frc2017.auto.modes;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.auto.AutoPathLoader;
import com.palyrobotics.frc2017.auto.modes.archive.CenterPegAutoMode.Alliance;
import com.palyrobotics.frc2017.behavior.ParallelRoutine;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.CANTalonRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.DrivePathRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.DriveSensorResetRoutine;
import com.palyrobotics.frc2017.behavior.routines.scoring.CustomPositioningSliderRoutine;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.util.archive.DriveSignal;
import com.team254.lib.trajectory.Path;

import java.util.ArrayList;

/**
 * Created by Nihar on 4/13/17.
 * Center peg autonomous
 * Uses motion profile for initial attempt, motion magic for backup attempts
 */
public class TrajectoryCenterPegAutoMode extends AutoModeBase {
	// Stores configuration
	private final Alliance mVariant;	// Alliance color
	private boolean mBackup = true;	// Whether to conduct additional scoring attempts
	private Path mPath;	// Motion profile path
	
	private final boolean mUseGyro = true;	// Whether to use gyro correction in motion profile

	private final Gains mShortGains;	// Motion magic gains for backup
	private final Gains.TrajectoryGains mTrajectoryGains;	// Motion profile gains
	
	private final double kBackupDistance = 15;	// distance in inches
	
	// Store the left/right slider positions
	private double[] mSliderPositions;
	private double[] kBlueSliderPositions = new double[]{2.5, 0, 4.5};
	private double[] kRedSliderPositions = new double[]{0, 2.5, -3.5};

	private final double kPilotWaitTime = 1.5;	// time in seconds

	private SequentialRoutine mSequentialRoutine;
	
	/**
	 * Constructor
	 * @param variant Alliance color
	 * @param backup Whether to conduct additional scoring attempts
	 */
	public TrajectoryCenterPegAutoMode(Alliance variant, boolean backup) {
		AutoPathLoader.loadPaths();
		mVariant = variant;
		mBackup = backup;
		mTrajectoryGains = Gains.kTrajectoryStraightGains;
		switch (mVariant) {
			case BLUE:
				mSliderPositions = kBlueSliderPositions;
				break;
			case RED:
				mSliderPositions = kRedSliderPositions;
				break;
		}
		mShortGains = Gains.steikShortDriveMotionMagicGains;
	}

	/**
	 * Creates a SequentialRoutine that represents a series of autonomous actions
	 */
	@Override
	public void prestart() {
		ArrayList<Routine> sequence = new ArrayList<>();
		
		sequence.add(new DriveSensorResetRoutine());
		switch (mVariant) {
		case BLUE:
			mPath = AutoPathLoader.get("BlueCenter");
			break;
		case RED:
			mPath = AutoPathLoader.get("RedCenter");
			break;
		}

		// Drive forward while moving slider to initial position
		ArrayList<Routine> initialSlide = new ArrayList<>();
		initialSlide.add(new CustomPositioningSliderRoutine(mSliderPositions[0]));
		initialSlide.add(new DrivePathRoutine(mPath, mTrajectoryGains, mUseGyro, false));
		sequence.add(new ParallelRoutine(initialSlide));

		// Wait for pilot and reset sensors
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		sequence.add(new DriveSensorResetRoutine());
	
		// Add second and third scoring attempts if specified
		if (mBackup) {
			sequence.add(getBackup(mSliderPositions[1]));
			sequence.add(new TimeoutRoutine(1.5));
			sequence.add(getBackup(mSliderPositions[2]));
			sequence.add(new TimeoutRoutine(kPilotWaitTime));
		}
		
		mSequentialRoutine = new SequentialRoutine(sequence);
	}

	/**
	 * @return The SequentialRoutine constructed in prestart()
	 */
	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}
	/*
	 * GET BACKUP
	 */
	private SequentialRoutine getBackup(double sliderPosition) {
		DriveSignal driveBackup = DriveSignal.getNeutralSignal();
		DriveSignal driveReturn = DriveSignal.getNeutralSignal();

		double driveBackupSetpoint = -kBackupDistance * Constants.kDriveTicksPerInch;
		driveBackup.leftMotor.setMotionMagic(driveBackupSetpoint, mShortGains, 
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveBackup.rightMotor.setMotionMagic(driveBackupSetpoint, mShortGains, 
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);

		// drive forward same distance as backup
		driveReturn.leftMotor.setMotionMagic(-driveBackupSetpoint+2*Constants.kDriveTicksPerInch, mShortGains,
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveReturn.rightMotor.setMotionMagic(-driveBackupSetpoint+2*Constants.kDriveTicksPerInch, mShortGains,
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		
		// Create a routine that drives back, then moves the slider while moving back forward
		ArrayList<Routine> sequence = new ArrayList<>();
		ArrayList<Routine> parallelSliding = new ArrayList<>();
		parallelSliding.add(new CANTalonRoutine(driveBackup, true));
		ArrayList<Routine> slideSequence = new ArrayList<>();
		slideSequence.add(new TimeoutRoutine(0.3));
		slideSequence.add(new CustomPositioningSliderRoutine(sliderPosition));
		parallelSliding.add(new SequentialRoutine(slideSequence));
		sequence.add(new ParallelRoutine(parallelSliding));
		sequence.add(new CANTalonRoutine(driveReturn, true, 1));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		return new SequentialRoutine(sequence);
	}

	@Override
	public String toString() {
		return "TrajectoryCenterPegAutoMode";
	}
}

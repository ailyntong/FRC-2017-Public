package com.palyrobotics.frc2017.auto.modes;

import java.util.ArrayList;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.auto.AutoPathLoader;
import com.palyrobotics.frc2017.auto.modes.SidePegAutoMode.SideAutoVariant;
import com.palyrobotics.frc2017.behavior.ParallelRoutine;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.SpatulaDownAutocorrectRoutine;
import com.palyrobotics.frc2017.behavior.routines.SpatulaUpRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.*;
import com.palyrobotics.frc2017.behavior.routines.scoring.CustomPositioningSliderRoutine;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.util.archive.DriveSignal;
import com.team254.lib.trajectory.Path;

/**
 * Side peg autonomous using motion profiles
 * @author Ailyn Tong
 */
public class TrajectorySidePegAutoMode extends AutoModeBase {
	// Desired action after initial attempt
	public enum TrajectorySidePostVariant {
		NONE,
		BACKUP,
		NEUTRAL_ZONE,
		BOTH
	}
	// Store configuration on construction
	private final SideAutoVariant mVariant;	// Alliance color and side
	private final TrajectorySidePostVariant mPostVariant;	// Post-score action
	private Path mPath, mPostPath;	// Scoring path and neutral zone path
	
	private final boolean mUseGyro = true;	// Whether to use gyro correction in motion profile
	private boolean mPostInverted;	// Left/right inversion of neutral zone path
	
	private final Gains mShortGains;	// Motion magic gains for backup
	private final Gains.TrajectoryGains mTrajectoryGains;	// Motion profile gains
	private final double kBackupDistance = 15;	// distance in inches
	private final double kPilotWaitTime = 1.5;	// time in seconds

	private double[] mSliderPositions;	// slider positions in inches for each backup attempt

	private SequentialRoutine mSequentialRoutine;
	
	/**
	 * Constructor
	 * @param direction Alliance color and side
	 * @param postScore Desired action after initial attempt
	 */
	public TrajectorySidePegAutoMode(SideAutoVariant direction, TrajectorySidePostVariant postScore) {
		AutoPathLoader.loadPaths();
		mVariant = direction;
		// Initialize scoring path and slider positions
		switch (mVariant) {
			case BLUE_BOILER:
				mPath = AutoPathLoader.get("BlueBoiler");
				mSliderPositions = new double[]{0, 3, -3};
				mPostInverted = true;
				break;
			case BLUE_LOADING:
				mPath = AutoPathLoader.get("BlueLoading");
				mSliderPositions = new double[]{0, 3, -3};
				mPostInverted = false;
				break;
			case RED_LOADING:
				mPath = AutoPathLoader.get("RedLoading");
				mSliderPositions = new double[]{0, 3, -3};
				mPostInverted = true;
				break;
			case RED_BOILER:
				mPath = AutoPathLoader.get("RedBoiler");
				mSliderPositions = new double[]{0, 3, -3};
				mPostInverted = false;
				break;
		}
		mPostVariant = postScore;
		// Initialize gains
		mTrajectoryGains = Gains.kTrajectoryGains;
		mShortGains = Gains.steikShortDriveMotionMagicGains;
	}

	/**
	 * Creates a SequentialRoutine that represents a series of autonomous actions
	 */
	@Override
	public void prestart() {
		ArrayList<Routine> sequence = new ArrayList<>();
		
		sequence.add(new DriveSensorResetRoutine());
		
		// Drive to peg while moving slider to initial position
		ArrayList<Routine> parallelSlider = new ArrayList<>();
		parallelSlider.add(new CustomPositioningSliderRoutine(mSliderPositions[0]));
		parallelSlider.add(new DrivePathRoutine(mPath, mTrajectoryGains, mUseGyro, false));
		sequence.add(new ParallelRoutine(parallelSlider));
		sequence.add(new DriveSensorResetRoutine());
		
		//Wait for pilot and reset sensors
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		sequence.add(new DriveSensorResetRoutine());
		
		// Add appropriate post-score action
		switch (mPostVariant) {
		case NONE:	// Do nothing
			mPostPath = null;
			break;
		case BACKUP:	// Second and third attemps
			mPostPath = null;
			sequence.add(getBackup(mSliderPositions[1]));
			sequence.add(getBackup(mSliderPositions[2]));
			break;
		case NEUTRAL_ZONE:	// Drop gear and drive to neutral zone
			sequence.add(getDrop());
			mPostPath = AutoPathLoader.get("RightSideDriveToNeutral");
			sequence.add(new DrivePathRoutine(mPostPath, mTrajectoryGains, mUseGyro, mPostInverted));
			break;
		case BOTH:	// Backup, then neutral zone
			mPostPath = AutoPathLoader.get("RightSideDriveToNeutral");
			sequence.add(getBackup(mSliderPositions[1]));
			sequence.add(getDrop());
			sequence.add(new DrivePathRoutine(mPostPath, mTrajectoryGains, mUseGyro, mPostInverted));
			break;
		}
		
		mSequentialRoutine = new SequentialRoutine(sequence);
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
		slideSequence.add(new TimeoutRoutine(0.5));
		slideSequence.add(new CustomPositioningSliderRoutine(sliderPosition));
		parallelSliding.add(new SequentialRoutine(slideSequence));
		sequence.add(new ParallelRoutine(parallelSliding));
		sequence.add(new CANTalonRoutine(driveReturn, true, 1));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		return new SequentialRoutine(sequence);
	}
	/*
	 * GET DROP
	 */
	private SequentialRoutine getDrop() {
		DriveSignal driveBackup = DriveSignal.getNeutralSignal();
		double driveBackupSetpoint = -30 * Constants.kDriveTicksPerInch;
		driveBackup.leftMotor.setMotionMagic(driveBackupSetpoint, mShortGains,
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveBackup.rightMotor.setMotionMagic(driveBackupSetpoint, mShortGains,
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);

		ArrayList<Routine> sequence = new ArrayList<>();
		ArrayList<Routine> parallelDrop = new ArrayList<>();
		ArrayList<Routine> spatulaSequence = new ArrayList<>();

		// Backup while dropping spatula, then turn 180 degrees and raise spatula
		parallelDrop.add(new CANTalonRoutine(driveBackup, true));
		spatulaSequence.add(new TimeoutRoutine(1));
		spatulaSequence.add(new SpatulaDownAutocorrectRoutine());
		parallelDrop.add(new SequentialRoutine(spatulaSequence));
		sequence.add(new ParallelRoutine(parallelDrop));
		sequence.add(new EncoderTurnAngleRoutine(180));
		sequence.add(new SpatulaUpRoutine());

		return new SequentialRoutine(sequence);
	}

	/**
	 * @return The SequentialRoutine constructed in prestart()
	 */
	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}

	@Override
	public String toString() {
		return "TrajectorySidePegAuto"+mVariant+mPostVariant;
	}
}

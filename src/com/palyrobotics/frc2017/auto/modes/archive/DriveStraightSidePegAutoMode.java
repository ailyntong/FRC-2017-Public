package com.palyrobotics.frc2017.auto.modes.archive;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.auto.modes.SidePegAutoMode;
import com.palyrobotics.frc2017.behavior.ParallelRoutine;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.SpatulaDownAutocorrectRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.*;
import com.palyrobotics.frc2017.behavior.routines.scoring.CustomPositioningSliderRoutine;
import com.palyrobotics.frc2017.config.AutoDistances;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.util.logger.Logger;

import java.util.ArrayList;

/**
 * Side peg autonomous using drive straight and gyro turn angle
 * Includes backup or drive to neutral zone
 * @author Ailyn Tong
 */
public class DriveStraightSidePegAutoMode extends AutoModeBase {
	// Store configuration on construction
	private final SidePegAutoMode.SideAutoVariant mVariant;
	private final SidePegAutoMode.SideAutoPostVariant mPostVariant;
	
	private SequentialRoutine mSequentialRoutine;

	private final double kPilotWaitTime = 2.5; // time in seconds
	private final double kBackupDistance = 10;	// distance in inches
	private final double kNeutralZoneDistance = 12 * 14;	// distance in inches

	private double mInitialSliderPosition = 0;	// slider position in inches
	private double mBackupPosition = 0;	// slider position in inches

	/**
	 * Constructor
	 * @param direction Alliance color and side of target peg
	 * @param postScore Desired action after initial attempt
	 */
	public DriveStraightSidePegAutoMode(SidePegAutoMode.SideAutoVariant direction, SidePegAutoMode.SideAutoPostVariant postScore) {
		mVariant = direction;
		mPostVariant = postScore;
	}

	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}

	@Override
	public void prestart() {
		System.out.println("Starting " + this.toString() + " Auto Mode");
		Logger.getInstance().logRobotThread("Starting " + this.toString() + " Auto Mode");

		ArrayList<Routine> sequence = new ArrayList<>();

		sequence.add(getDriveForward());

		// Turn angle and initialize backup slider position
		// NOTE: switch case falling, split by lefts vs rights
		switch (mVariant) {
		case RED_LOADING:
			mBackupPosition = 3;
			sequence.add(new GyroMotionMagicTurnAngleRoutine(Constants.kSidePegTurnAngleDegrees));
			break;
		case BLUE_BOILER:
			mBackupPosition = 1; //-3
			sequence.add(new GyroMotionMagicTurnAngleRoutine(Constants.kSidePegTurnAngleDegrees));
			break;
		case RED_BOILER:
			mBackupPosition = 2;
			sequence.add(new GyroMotionMagicTurnAngleRoutine(-Constants.kSidePegTurnAngleDegrees));
			break;
		case BLUE_LOADING:
			mBackupPosition = -4;
			sequence.add(new GyroMotionMagicTurnAngleRoutine(-Constants.kSidePegTurnAngleDegrees));
			break;
		}
		
		sequence.add(getDriveToAirship());
		sequence.add(new TimeoutRoutine(kPilotWaitTime));	// Wait for pilot to pull gear out

		// Add additional routines based on desired post-score action
		switch (mPostVariant) {
		case NONE:
			break;
		case BACKUP:
			sequence.add(getBackup(mBackupPosition));
			break;
		case NEUTRAL_ZONE:
			sequence.add(getDriveToNeutralZone());
			break;
		}

		mSequentialRoutine = new SequentialRoutine(sequence);
	}
	/*
	 * DRIVE FORWARD
	 */
	private Routine getDriveForward() {
		// For Red Left = Blue Right, Red Right = Blue Left
		double driveForwardSetpoint;
		switch (mVariant) {
		// loading station side
		case RED_LOADING:
			mInitialSliderPosition = 0;
			driveForwardSetpoint = AutoDistances.kRedLoadingStationForwardDistanceInches;
			break;
		case BLUE_LOADING:
			mInitialSliderPosition = -1.5;
			driveForwardSetpoint = AutoDistances.kBlueLoadingStationForwardDistanceInches;
			break;
		// boiler side
		case RED_BOILER:
			mInitialSliderPosition = 0;
			driveForwardSetpoint = AutoDistances.kRedBoilerForwardDistanceInches;
			break;
		case BLUE_BOILER:
			mInitialSliderPosition = 2.5;
			driveForwardSetpoint = AutoDistances.kBlueBoilerForwardDistanceInches;
			break;
		default:
			System.err.println("What in tarnation no side peg distance");
			driveForwardSetpoint = 0;
			break;
		}
		
		Logger.getInstance().logRobotThread("Drive forward", driveForwardSetpoint);
		
		// Drive forward while moving slider to initial position
		ArrayList<Routine> initialSlide = new ArrayList<>();
		initialSlide.add(new DriveStraightRoutine(driveForwardSetpoint));
		initialSlide.add(new CustomPositioningSliderRoutine(mInitialSliderPosition));
		return new ParallelRoutine(initialSlide);
	}
	/*
	 * GET AIRSHIP
	 */
	private DriveStraightRoutine getDriveToAirship() {
		double driveToAirshipSetpoint = 0;
		switch (mVariant) {
		// loading station side
		case RED_LOADING:
			driveToAirshipSetpoint = AutoDistances.kRedLoadingStationAirshipDistanceInches;
			break;
		case BLUE_LOADING:
			driveToAirshipSetpoint = AutoDistances.kBlueLoadingStationAirshipDistanceInches;
			break;
		// boiler side
		case RED_BOILER:
			driveToAirshipSetpoint = AutoDistances.kRedBoilerAirshipDistanceInches;
			break;
		case BLUE_BOILER:
			driveToAirshipSetpoint = AutoDistances.kBlueBoilerAirshipDistanceInches;
			break;
		default:
			System.err.println("What in tarnation no side peg airship distance");
			driveToAirshipSetpoint = 0;
			break;
		}
		driveToAirshipSetpoint += 2 * Constants.kDriveTicksPerInch;	// Drive extra to ensure gear is on peg
		
		Logger.getInstance().logRobotThread("Drive to airship", driveToAirshipSetpoint);
		return new DriveStraightRoutine(driveToAirshipSetpoint);
	}
	/*
	 * GET BACKUP
	 */
	private SequentialRoutine getBackup(double sliderPosition) {
		double driveBackupSetpoint = -kBackupDistance;
		
		// Create a routine that drives back, then moves the slider while moving back forward
		ArrayList<Routine> sequence = new ArrayList<>();
		ArrayList<Routine> parallelSliding = new ArrayList<>();
		parallelSliding.add(new DriveStraightRoutine(driveBackupSetpoint));
		ArrayList<Routine> slideSequence = new ArrayList<>();
		slideSequence.add(new TimeoutRoutine(0.5));
		slideSequence.add(new CustomPositioningSliderRoutine(sliderPosition));
		parallelSliding.add(new SequentialRoutine(slideSequence));
		sequence.add(new ParallelRoutine(parallelSliding));
		sequence.add(new DriveStraightRoutine(-driveBackupSetpoint + 3));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		return new SequentialRoutine(sequence);
	}
	/*
	 * GET NEUTRAL ZONE
	 */
	private SequentialRoutine getDriveToNeutralZone() {
		double driveBackupSetpoint = -(kBackupDistance + 12);
		double driveToNeutralZoneSetpoint = kNeutralZoneDistance;
		
		ArrayList<Routine> sequence = new ArrayList<>();
		
		// Back up while lowering spatula
		ArrayList<Routine> parallel = new ArrayList<>();
		parallel.add(new DriveStraightRoutine(driveBackupSetpoint));
		ArrayList<Routine> spatulaSequence = new ArrayList<>();
		spatulaSequence.add(new TimeoutRoutine(1));
		spatulaSequence.add(new SpatulaDownAutocorrectRoutine());
		parallel.add(new SequentialRoutine(spatulaSequence));
		sequence.add(new ParallelRoutine(parallel));

		// Turn towards neutral zone
		// Intentional switch-case falling
		switch (mVariant) {
		case RED_LOADING:
		case BLUE_BOILER:
			sequence.add(new GyroMotionMagicTurnAngleRoutine(-Constants.kSidePegTurnAngleDegrees));
			break;
		case RED_BOILER:
		case BLUE_LOADING:
			sequence.add(new GyroMotionMagicTurnAngleRoutine(Constants.kSidePegTurnAngleDegrees));
			break;
		}
		
		// Drive forward
		sequence.add(new DriveStraightRoutine(driveToNeutralZoneSetpoint));
		
		return new SequentialRoutine(sequence);
	}

	@Override
	public String toString() {
		String name;
		switch (mVariant) {
		case RED_LOADING:
			name = "RedLeftDriveStraingtSidePeg";
			break;
		case RED_BOILER:
			name = "RedRightDriveStraightSidePeg";
			break;
		case BLUE_BOILER:
			name = "BlueLeftDriveStraightSidePeg";
			break;
		case BLUE_LOADING:
			name = "BlueRightDriveStraightSidePeg";
			break;
		default:
			name = "DriveStraightSidePeg";
			break;
		}
		name += "SliderInitialMove" + mInitialSliderPosition;
		name += "GyroTurn";
		switch (mPostVariant) {
		case NONE:
			break;
		case BACKUP:
			name += "Backup" + mBackupPosition;
			break;
		case NEUTRAL_ZONE:
			name += "NeutralZone";
			break;
		}
		return name;
	}
}
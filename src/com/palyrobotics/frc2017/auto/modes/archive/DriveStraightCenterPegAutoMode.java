package com.palyrobotics.frc2017.auto.modes.archive;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.auto.modes.archive.CenterPegAutoMode;
import com.palyrobotics.frc2017.behavior.ParallelRoutine;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.SpatulaDownAutocorrectRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.DriveStraightRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.GyroMotionMagicTurnAngleRoutine;
import com.palyrobotics.frc2017.behavior.routines.scoring.CustomPositioningSliderRoutine;
import com.palyrobotics.frc2017.config.AutoDistances;
import com.palyrobotics.frc2017.util.logger.Logger;

import java.util.ArrayList;

/**
 * Center peg autonomous using drive straight and gyro turn angle
 * Includes backup or drive to neutral zone
 * @author Ailyn Tong
 */
public class DriveStraightCenterPegAutoMode extends AutoModeBase {
	// Represents post score action
	public enum CenterAutoPostVariant {
		NONE,
		BACKUP,
		NEUTRAL_ZONE_LEFT,
		NEUTRAL_ZONE_RIGHT
	}
	
	// Store configuration on construction
	private final CenterPegAutoMode.Alliance mAlliance;
	private final CenterAutoPostVariant mVariant;
	
	private SequentialRoutine mSequentialRoutine;
	
	private double mInitialSliderPosition;	// distance from center in inches
	private final double kBackupDistance = 10;	// distance in inches
	private final double kPilotWaitTime = 2;	// time in seconds
	
	// Drop and drive distances
	private final double kClearAirshipDistance = 5 * 14;	// distance in inches
	private final double kNeutralZoneDistance = 12 * 14;	// distance in inches

	/**
	 * Constructor
	 * @param alliance Red or blue side
	 * @param variant Desired action after initial attempt
	 */
	public DriveStraightCenterPegAutoMode(CenterPegAutoMode.Alliance alliance, CenterAutoPostVariant variant) {
		mAlliance = alliance;
		mVariant = variant;
	}

	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}

	@Override
	public void prestart() {
		System.out.println("Starting " + this.toString() + " Auto Mode");
		Logger.getInstance().logRobotThread("Starting " + this.toString() + " Auto Mode");
		// Construct sequence of routines to run
		ArrayList<Routine> sequence = new ArrayList<>();
		// Straight drive distance to the center peg
		double driveForwardSetpoint =
				((mAlliance == CenterPegAutoMode.Alliance.BLUE) ? AutoDistances.kBlueCenterPegDistanceInches : AutoDistances.kRedCenterPegDistanceInches);
		
		mInitialSliderPosition = (mAlliance == CenterPegAutoMode.Alliance.BLUE) ? -2.5 : 0;
		
		// Drive forward while moving slider to initial position
		ArrayList<Routine> initialSlide = new ArrayList<>();
		initialSlide.add(new DriveStraightRoutine(driveForwardSetpoint));
		initialSlide.add(new CustomPositioningSliderRoutine(mInitialSliderPosition));
		sequence.add(new ParallelRoutine(initialSlide));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		// Add additional routines based on desired post-score action
		switch (mVariant) {
		case NONE:
			break;
		case BACKUP:
			// Backup, drive forward while repositioning slider, and wait for pilot
			double backup = (mAlliance == CenterPegAutoMode.Alliance.BLUE) ? 0 : 5;
			sequence.add(getBackup(backup));
			sequence.add(new TimeoutRoutine(kPilotWaitTime));
			break;
		case NEUTRAL_ZONE_LEFT:
			// Drive around left side of airship towards neutral zone
			sequence.add(getDriveToNeutralZone(-90));
			break;
		case NEUTRAL_ZONE_RIGHT:
			// Drive around right side of airship towards neutral zone
			sequence.add(getDriveToNeutralZone(90));
			break;
		}

//		for (Routine r : sequence) {
//			System.out.println(r.getName());
//		}

		mSequentialRoutine = new SequentialRoutine(sequence);
	}
	
	@Override
	public String toString() {
		String name = (mAlliance == CenterPegAutoMode.Alliance.BLUE) ? "BlueDriveStraightCenterPeg" : "RedDriveStraightCenterPeg";
		switch (mVariant) {
		case NONE:
			break;
		case BACKUP:
			name += "Backup";
			break;
		case NEUTRAL_ZONE_LEFT:
			name += "NeutralZoneLeft";
			break;
		case NEUTRAL_ZONE_RIGHT:
			name += "NeutralZoneRight";
		}
		return name;
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
		sequence.add(new DriveStraightRoutine(-driveBackupSetpoint + 3));	// Drive extra to ensure gear is on peg
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		return new SequentialRoutine(sequence);
	}
	/*
	 * GET NEUTRAL ZONE
	 */
	private SequentialRoutine getDriveToNeutralZone(double angle) {
		double driveBackupSetpoint = -(kBackupDistance + 12);
		double driveClearAirshipSetpoint = kClearAirshipDistance;
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
		// Turn parallel to airship
		sequence.add(new GyroMotionMagicTurnAngleRoutine(angle));
		// Drive clear of airship
		sequence.add(new DriveStraightRoutine(driveClearAirshipSetpoint));
		// Turn to face neutral zone
		sequence.add(new GyroMotionMagicTurnAngleRoutine(-angle));
		// Drive forward
		sequence.add(new DriveStraightRoutine(driveToNeutralZoneSetpoint));
		
		return new SequentialRoutine(sequence);
	}
}
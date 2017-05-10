package com.palyrobotics.frc2017.auto.modes;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.auto.modes.SidePegAutoMode;
import com.palyrobotics.frc2017.auto.modes.SidePegAutoMode.SideAutoPostVariant;
import com.palyrobotics.frc2017.behavior.ParallelRoutine;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.*;
import com.palyrobotics.frc2017.behavior.routines.scoring.CustomPositioningSliderRoutine;
import com.palyrobotics.frc2017.behavior.routines.scoring.VisionSliderRoutine;
import com.palyrobotics.frc2017.config.AutoDistances;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.config.Gains;
import com.palyrobotics.frc2017.util.archive.DriveSignal;
import com.palyrobotics.frc2017.util.logger.Logger;
import com.palyrobotics.frc2017.vision.AndroidConnectionHelper;

import java.util.ArrayList;

/**
 * Created by Nihar on 2/11/17.
 * Goes for side peg autonomous
 * Can set initial slider position and a backup slider position (absolute, 
 * not relative to the vision point)
 */
public class VisionSidePegAutoMode extends AutoModeBase {

	// Store configuration on construction
	private final SidePegAutoMode.SideAutoVariant mVariant;	// ALliance color and side
	private final boolean mBackup;	// Whether to conduct additional scoring attempts
	
	private Routine mSequentialRoutine;

	// Long distance vs short distance
	private Gains mLongGains, mShortGains;

	private final double kPilotWaitTime = 1.5; // time in seconds
	private final double kBackupDistance = 12; // distance in inches
	private double kOvershootDistance = 0;
	private double kBonusDistance = 20; // extra space

	// Slider positions for each scoring attempt
	private double[] mSliderPositions = new double[2];

	/**
	 * Constructor
	 * @param direction Alliance color and side
	 * @param backup Whether to conduct additional scoring attempts
	 */
	public VisionSidePegAutoMode(SidePegAutoMode.SideAutoVariant direction, boolean backup) {
		mVariant = direction;
		mBackup = backup;
		// Initialize gains
		mLongGains = Gains.steikLongDriveMotionMagicGains;
		mShortGains = Gains.steikShortDriveMotionMagicGains;
		// Initialize slider positions
		mSliderPositions = new double[2];
		mSliderPositions[0] = -7;
		switch (mVariant) {
			// loading station
			case RED_LOADING:
				mSliderPositions[1] = 2;
				break;
			case BLUE_LOADING:
				mSliderPositions[1] = 2;
				break;
			// boiler side
			case RED_BOILER:
				mSliderPositions[1] = 0;
				break;
			case BLUE_BOILER:
				mSliderPositions[1] = 1;
				break;
		}

	}

	/**
	 * @return The SequentialRoutine constructed in prestart()
	 */
	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}

	/**
	 * Creates a SequentialRoutine that represents a series of autonomous actions
	 */
	@Override
	public void prestart() {
		if(AndroidConnectionHelper.getInstance().isServerStarted()){
			System.out.println("Failed to find vision server, revert auto");
		}
		System.out.println("Starting "+this.toString()+" Auto Mode");
		Logger.getInstance().logRobotThread("Starting "+this.toString()+" Auto Mode");

		// Revert to motion magic side peg if vision app fails to connect properly
		if (!AndroidConnectionHelper.getInstance().isServerStarted() || !AndroidConnectionHelper.getInstance().isNexusConnected()) {
			System.out.println("Vision server not started!");
			Logger.getInstance().logRobotThread("Vision server not detected, fallback to default side peg");
			SidePegAutoMode backup = new SidePegAutoMode(mVariant, SideAutoPostVariant.BACKUP);
			backup.prestart();
			mSequentialRoutine = backup.getRoutine();
			return;
		}
		ArrayList<Routine> sequence = new ArrayList<>();

		sequence.add(getDriveForward());

		// NOTE: switch case falling, split by lefts vs rights
		switch (mVariant) {
		// loading station
		case RED_LOADING:
			sequence.add(new EncoderTurnAngleRoutine(Constants.kSidePegTurnAngleDegrees));
			break;
		case BLUE_LOADING:
			sequence.add(new EncoderTurnAngleRoutine(-Constants.kSidePegTurnAngleDegrees));
			break;
		// boiler side
		case RED_BOILER:
			sequence.add(new EncoderTurnAngleRoutine(-Constants.kSidePegTurnAngleDegrees));
			break;
		case BLUE_BOILER:
			sequence.add(new EncoderTurnAngleRoutine(Constants.kSidePegTurnAngleDegrees));
			break;
		}
		
		sequence.add(getDriveToAirship());
		sequence.add(getFirstAttempt());
		sequence.add(new TimeoutRoutine(kPilotWaitTime));	// Wait so pilot can pull gear out
		if (mBackup) {
			sequence.add(getBackup(mSliderPositions[1]));
		}

		mSequentialRoutine = new SequentialRoutine(sequence);
	}
	/*
	 * DRIVE FORWARD
	 */
	private Routine getDriveForward() {
		DriveSignal driveForward = DriveSignal.getNeutralSignal();
		// For Red Left = Blue Right, Red Right = Blue Left
		double driveForwardSetpoint;
		switch (mVariant) {
		// loading station side
		case RED_LOADING:
			driveForwardSetpoint = AutoDistances.kRedLoadingStationForwardDistanceInches * Constants.kDriveTicksPerInch;
			break;
		case BLUE_LOADING:
			driveForwardSetpoint = AutoDistances.kBlueLoadingStationForwardDistanceInches * Constants.kDriveTicksPerInch;
			break;
			// boiler side
		case RED_BOILER:
			driveForwardSetpoint = AutoDistances.kRedBoilerForwardDistanceInches * Constants.kDriveTicksPerInch;
			break;
		case BLUE_BOILER:
			driveForwardSetpoint = AutoDistances.kBlueBoilerForwardDistanceInches * Constants.kDriveTicksPerInch;
			break;
		default:
			System.err.println("What in tarnation no side peg distance");
			driveForwardSetpoint = 0;
			break;
		}
		driveForwardSetpoint += kOvershootDistance;
		driveForward.leftMotor.setMotionMagic(driveForwardSetpoint, mLongGains,
				Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		driveForward.rightMotor.setMotionMagic(driveForwardSetpoint, mLongGains,
				Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		
		// Drive forward while moving slider to initial position
		Logger.getInstance().logRobotThread("Drive forward", driveForward);
		ArrayList<Routine> initialSlide = new ArrayList<>();
		initialSlide.add(new CANTalonRoutine(driveForward, true));
		initialSlide.add(new CustomPositioningSliderRoutine(mSliderPositions[0]));
		return new ParallelRoutine(initialSlide);
	}
	/*
	 * GET AIRSHIP
	 */
	private CANTalonRoutine getDriveToAirship() {
		DriveSignal driveToAirship = DriveSignal.getNeutralSignal();
		double driveToAirshipSetpoint = 0;
		switch (mVariant) {
		// loading station side
		case RED_LOADING:
			driveToAirshipSetpoint = AutoDistances.k254LoadingStationAirshipDistanceInches * Constants.kDriveTicksPerInch;
			break;
		case BLUE_LOADING:
			driveToAirshipSetpoint = AutoDistances.k254LoadingStationAirshipDistanceInches * Constants.kDriveTicksPerInch;
			break;
			// boiler side
		case RED_BOILER:
			driveToAirshipSetpoint = AutoDistances.k254BoilerAirshipDistanceInches * Constants.kDriveTicksPerInch;
			break;
		case BLUE_BOILER:
			driveToAirshipSetpoint = AutoDistances.k254BoilerAirshipDistanceInches * Constants.kDriveTicksPerInch;
			break;
		default:
			System.err.println("What in tarnation no side peg airship distance");
			driveToAirshipSetpoint = 0;
			break;
		}
		// Stop before reaching airship
		driveToAirshipSetpoint -= kBonusDistance*Constants.kDriveTicksPerInch;
		driveToAirship.leftMotor.setMotionMagic(driveToAirshipSetpoint, mLongGains,
				Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		driveToAirship.rightMotor.setMotionMagic(driveToAirshipSetpoint, mLongGains,
				Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		
		Logger.getInstance().logRobotThread("Drive to airship", driveToAirship);
		return new CANTalonRoutine(driveToAirship, true, 5);
	}
	/*
	 * GET FIRST ATTEMPT
	 */
	private Routine getFirstAttempt() {
		double scoreSetpoint = kBonusDistance*Constants.kDriveTicksPerInch;
		scoreSetpoint += 2;
		DriveSignal driveScore = DriveSignal.getNeutralSignal();
		driveScore.leftMotor.setMotionMagic(scoreSetpoint, mShortGains,
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveScore.rightMotor.setMotionMagic(scoreSetpoint, mShortGains,
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		
		// Move slider using vision, then drive to peg
		ArrayList<Routine> scoreSequence = new ArrayList<>();
		scoreSequence.add(new VisionSliderRoutine());
		scoreSequence.add(new CANTalonRoutine(driveScore, true));
		return new SequentialRoutine(scoreSequence);
	}
	/*
	 * GET BACKUP
	 */
	private Routine getBackup(double sliderPosition) {
		DriveSignal driveBackup = DriveSignal.getNeutralSignal();
		DriveSignal driveReturn = DriveSignal.getNeutralSignal();

		double driveBackupSetpoint = -kBackupDistance * Constants.kDriveTicksPerInch;
		driveBackup.leftMotor.setMotionMagic(driveBackupSetpoint, mShortGains, 
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveBackup.rightMotor.setMotionMagic(driveBackupSetpoint, mShortGains, 
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);

		// drive forward same distance as backup
		driveReturn.leftMotor.setMotionMagic(-driveBackupSetpoint+3*Constants.kDriveTicksPerInch, mShortGains, 
				Gains.kSteikShortDriveMotionMagicCruiseVelocity, Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveReturn.rightMotor.setMotionMagic(-driveBackupSetpoint+3*Constants.kDriveTicksPerInch, mShortGains, 
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
		sequence.add(new CANTalonRoutine(driveReturn, true, 2));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		return new SequentialRoutine(sequence);
	}

	@Override
	public String toString() {
		String name;
		name = "Vision";
		switch (mVariant) {
		case RED_LOADING:
			name += "RedLeftSidePeg";
			break;
		case RED_BOILER:
			name += "RedRightSidePeg";
			break;
		case BLUE_BOILER:
			name += "BlueLeftSidePeg";
			break;
		case BLUE_LOADING:
			name += "BlueRightSidePeg";
			break;
		default:
			name += "SidePeg";
			break;
		}
		name += "SliderInitialMove"+mSliderPositions[0];
		name += "EncoderTurn";
		if (mBackup) {
			name += "Backup"+mSliderPositions[1];
		} else {
			name += "NotBackup";
		}
		return name;
	}
}
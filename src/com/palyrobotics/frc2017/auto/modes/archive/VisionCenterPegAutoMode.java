package com.palyrobotics.frc2017.auto.modes.archive;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.auto.modes.archive.CenterPegAutoMode;
import com.palyrobotics.frc2017.behavior.ParallelRoutine;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.TimeoutRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.CANTalonRoutine;
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
 * BBTurnAngle might be replaced with EncoderTurnAngle if no gyro
 */
public class VisionCenterPegAutoMode extends AutoModeBase {
	// Store configuration on construction
	private final CenterPegAutoMode.Alliance mAlliance;
	private Routine mSequentialRoutine;
	private boolean mBackup = true;
	
	private Gains mShortGains, mLongGains;
	private double mInitialSliderPosition;	// distance from center in inches
	private final double kBackupDistance = 10;	// distance in inches
	private final double kPilotWaitTime = 3;	// time in seconds
	private double kBonusDistance = 14;	// distance in inches

	/**
	 * Constructor
	 * @param alliance Red or blue side
	 * @param isRightTarget Which target the vision app is tracking
	 * @param backup Whether to conduct a second scoring attempt
	 */
	public VisionCenterPegAutoMode(CenterPegAutoMode.Alliance alliance, boolean isRightTarget, boolean backup) {
		mAlliance = alliance;
		mInitialSliderPosition = (isRightTarget) ? -7 : 7;	// Slider should not block vision
		mShortGains = Gains.steikShortDriveMotionMagicGains;
		mLongGains = Gains.steikLongDriveMotionMagicGains;

		mBackup = backup;
	}

	@Override
	public Routine getRoutine() {
		return mSequentialRoutine;
	}

	@Override
	public void prestart() {
		String log = "Starting Vision Center Peg Auto Mode";
		Logger.getInstance().logRobotThread("Starting Vision Center Peg Auto Mode");
		// Runs regular center peg autonomous if vision app is not connected
		if (!AndroidConnectionHelper.getInstance().isServerStarted()  || !AndroidConnectionHelper.getInstance().isNexusConnected()) {
			System.out.println("Failed to find vision server, revert auto");
			Logger.getInstance().logRobotThread("Failed to find vision server, revert");
			CenterPegAutoMode fallback = new CenterPegAutoMode(mAlliance, true);
			fallback.prestart();
			mSequentialRoutine = fallback.getRoutine();
			return;
		}
		// Construct sequence of routines to run
		ArrayList<Routine> sequence = new ArrayList<>();
		// Straight drive distance to the center peg
		DriveSignal driveForward = DriveSignal.getNeutralSignal();
		double driveForwardSetpoint =
				((mAlliance == CenterPegAutoMode.Alliance.BLUE) ? AutoDistances.k254CenterPegDistanceInches : AutoDistances.k254CenterPegDistanceInches)
						* Constants.kDriveTicksPerInch;
		driveForwardSetpoint -= kBonusDistance * Constants.kDriveTicksPerInch;
		// Aegir: right +30
		// Vali: left +100
		driveForward.leftMotor.setMotionMagic(driveForwardSetpoint, mLongGains,
			Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		driveForward.rightMotor.setMotionMagic(driveForwardSetpoint+30, mLongGains,
				Gains.kSteikLongDriveMotionMagicCruiseVelocity, Gains.kSteikLongDriveMotionMagicMaxAcceleration);
		
		DriveSignal driveBonus = DriveSignal.getNeutralSignal();
		driveBonus.leftMotor.setMotionMagic(kBonusDistance, mShortGains, Gains.kSteikShortDriveMotionMagicCruiseVelocity,
				Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		driveBonus.rightMotor.setMotionMagic(kBonusDistance, mShortGains, Gains.kSteikShortDriveMotionMagicCruiseVelocity,
				Gains.kSteikShortDriveMotionMagicMaxAcceleration);
		
		// Drive forward while moving slider to initial position
		// Stop before reaching the peg, use vision to move slider, then move forward
		ArrayList<Routine> initialSlide = new ArrayList<>();
		initialSlide.add(new CANTalonRoutine(driveForward, true));
		initialSlide.add(new CustomPositioningSliderRoutine(mInitialSliderPosition));
		sequence.add(new ParallelRoutine(initialSlide));
		sequence.add(new VisionSliderRoutine());
		sequence.add(new CANTalonRoutine(driveBonus, true));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		// Backup, drive forward while repositioning slider, and wait for pilot
		if (mBackup) {
			sequence.add(getBackup(-2));
			sequence.add(new TimeoutRoutine(kPilotWaitTime));
		}

		mSequentialRoutine = new SequentialRoutine(sequence);
		System.out.println(log);
	}
	@Override
	public String toString() {
		String name = (mAlliance == CenterPegAutoMode.Alliance.BLUE) ? "BlueCenterPeg" : "RedCenterPeg";
		return "Vision"+name;
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
		sequence.add(new CANTalonRoutine(driveReturn, true));
		sequence.add(new TimeoutRoutine(kPilotWaitTime));
		
		return new SequentialRoutine(sequence);
	}
}
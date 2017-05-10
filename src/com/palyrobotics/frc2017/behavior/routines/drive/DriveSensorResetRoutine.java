package com.palyrobotics.frc2017.behavior.routines.drive;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.config.Constants;
import com.palyrobotics.frc2017.robot.HardwareAdapter;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Created by EricLiu on 4/13/17.
 * Reset drive sensors
 */
public class DriveSensorResetRoutine extends Routine {
	/**
	 * Reset sensors
	 */
    @Override
    public void start() {
        HardwareAdapter.getInstance().getDrivetrain().resetSensors();
    }

    /**
     * @return commands
     */
    @Override
    public Commands update(Commands commands) {
        Commands output = commands.copy();
        return output;
    }

    /**
     * @return commands
     */
    @Override
    public Commands cancel(Commands commands) {
        Commands output = commands.copy();
        return output;
    }

    /**
     * @return Whether the zeroed encoder and heading values are within tolerance
     */
    @Override
    public boolean finished() {
        if(Math.abs(drive.getPose().leftEnc) <= Constants.kAcceptableEncoderZeroError && Math.abs(drive.getPose().rightEnc) <= Constants.kAcceptableEncoderZeroError
                && Math.abs(drive.getPose().heading) <= Constants.kAcceptableGyroZeroError) {
            return true;
        } else return false;
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
        return "DriveSensorResetRoutine";
    }
}

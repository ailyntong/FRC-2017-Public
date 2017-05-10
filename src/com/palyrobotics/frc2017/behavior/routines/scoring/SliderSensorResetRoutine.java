package com.palyrobotics.frc2017.behavior.routines.scoring;

import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.config.Commands;
import com.palyrobotics.frc2017.robot.HardwareAdapter;
import com.palyrobotics.frc2017.util.Subsystem;

/**
 * Created by EricLiu on 4/17/17.
 * Reset slider sensors
 */
public class SliderSensorResetRoutine extends Routine {
	/**
	 * Reset slider sensors
	 */
    @Override
    public void start() {
        HardwareAdapter.getInstance().getSlider().resetEncoder();
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
     * @return true
     */
    @Override
    public boolean finished() {
        return true;
    }

    /**
     * @return Set of subsystems required by routine
     */
    @Override
    public Subsystem[] getRequiredSubsystems() {
        return new Subsystem[]{slider};
    }

    /**
     * @return Name of routine
     */
    @Override
    public String getName() {
        return "SliderSensorResetRoutine";
    }
}

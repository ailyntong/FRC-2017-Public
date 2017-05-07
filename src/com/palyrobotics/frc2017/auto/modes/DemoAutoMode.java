package com.palyrobotics.frc2017.auto.modes;

import com.palyrobotics.frc2017.auto.AutoModeBase;
import com.palyrobotics.frc2017.behavior.Routine;
import com.palyrobotics.frc2017.behavior.SequentialRoutine;
import com.palyrobotics.frc2017.behavior.routines.drive.EncoderTurnAngleRoutine;
import com.palyrobotics.frc2017.behavior.routines.scoring.CustomPositioningSliderRoutine;

import java.util.ArrayList;

/**
 * Created by Nihar on 5/7/17.
 * Spin in a circle, move the slider and climber
 */
public class DemoAutoMode extends AutoModeBase {
	@Override
	public String toString() {
		return "DemoAutoMode";
	}

	@Override
	public void prestart() {
		System.out.println("Starting demo auto mode!");
	}

	@Override
	public Routine getRoutine() {
		ArrayList<Routine> sequence = new ArrayList<>();
		sequence.add(new CustomPositioningSliderRoutine(-7));
		sequence.add(new CustomPositioningSliderRoutine(7));
		sequence.add(new EncoderTurnAngleRoutine(180));
		sequence.add(new EncoderTurnAngleRoutine(-180));
		return new SequentialRoutine(sequence);
	}
}

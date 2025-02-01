package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import edu.wpi.first.wpilibj2.command.Command

abstract class AutonomousSegment(val startingSide: ReefSide) {
	abstract fun generateCommand(): Command
}
package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command

/**
 * An abstract superclass representing a segment of an autonomous routine.
 *
 * @param startingSide - Side to start executing the segment's command from.
 * @param endingSide - The side in which the segment ends.
 * @param isClockwise - Whether the routine gets to [startingSide] by going clockwise around the reef or counterclockwise.
 */
abstract class AutonomousSegment(
	val startingSide: ReefSide,
	val endingSide: ReefSide,
	val isClockwise: Boolean,
) {
	abstract fun generateCommand(alliance: Alliance): Command
}
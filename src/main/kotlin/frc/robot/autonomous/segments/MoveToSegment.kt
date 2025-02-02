package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import com.hamosad1657.lib.commands.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command

/**
 * A segment which does nothing. Used to move the robot to another side of the reef.
 *
 * @param goalSide - The new side to move to.
 * @param isClockwise - Whether to get to the goal side by going clockwise around the reef or counterclockwise.
 */
class MoveToSegment(
	goalSide: ReefSide,
	isClockwise: Boolean,
): AutonomousSegment(goalSide, goalSide, isClockwise) {
	override fun generateCommand(alliance: Alliance): Command {
		return instantCommand {  }
	}
}
package frc.robot.autonomous

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import frc.robot.autonomous.segments.AutonomousSegment
import frc.robot.commands.*
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * Represents an autonomous routine for the robot to follow.
 *
 * @param startingPosition - The starting position (1 -> 6) the robot starts from.
 */
class AutonomousRoutine(
	val startingPosition: Int,
	val segments: List<AutonomousSegment>,
) {
	val startingSide = when (startingPosition) {
		1, 2, 3 -> if (segments[0].startingSide.sideNumber - )
	}
	fun generateCommand() = withName("Autonomous routine") {
		SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("Position $startingPosition to ")
		)
	}
}
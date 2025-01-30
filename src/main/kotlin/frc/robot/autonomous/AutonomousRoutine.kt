package frc.robot.autonomous

import frc.robot.autonomous.segments.AutonomousSegment

/**
 * Represents an autonomous routine for the robot to follow
 */
class AutonomousRoutine(
	val startingPosition: Int,
	val segments: List<AutonomousSegment>,
) {

}
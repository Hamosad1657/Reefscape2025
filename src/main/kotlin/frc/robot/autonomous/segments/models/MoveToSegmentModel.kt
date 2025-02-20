package frc.robot.autonomous.segments.models

import frc.robot.autonomous.segments.MoveToSegment

class MoveToSegmentModel: AutonomousSegmentModel() {
	override fun createSegment(): MoveToSegment {
		return MoveToSegment(startingSide, isClockwise)
	}
}
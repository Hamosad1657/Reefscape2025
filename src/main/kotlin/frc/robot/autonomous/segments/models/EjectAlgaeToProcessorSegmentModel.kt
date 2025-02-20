package frc.robot.autonomous.segments.models

import frc.robot.autonomous.segments.AutonomousSegment
import frc.robot.autonomous.segments.EjectAlgaeToProcessorSegment

class EjectAlgaeToProcessorSegmentModel: AutonomousSegmentModel() {
	override fun createSegment(): AutonomousSegment {
		return EjectAlgaeToProcessorSegment(startingSide, endingSide, isClockwise)
	}
}
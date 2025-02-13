package frc.robot.autonomous.segments.models

import com.fasterxml.jackson.annotation.JsonProperty
import frc.robot.autonomous.segments.CollectAlgaeSegment
import frc.robot.field.ReefAlgae

class CollectAlgaeSegmentModel: AutonomousSegmentModel() {
	@set:JsonProperty("AlgaeToCollect")
	var algaeToCollect: ReefAlgae = ReefAlgae.AB

	override fun createSegment(): CollectAlgaeSegment {
		return CollectAlgaeSegment(algaeToCollect, isClockwise)
	}
}
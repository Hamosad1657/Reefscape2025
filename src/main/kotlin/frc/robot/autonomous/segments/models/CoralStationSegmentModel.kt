package frc.robot.autonomous.segments.models

import com.fasterxml.jackson.annotation.JsonProperty
import frc.robot.autonomous.segments.CoralStationSegment
import frc.robot.field.CoralStation
import frc.robot.field.CoralStation.KL

class CoralStationSegmentModel: AutonomousSegmentModel() {
	@set:JsonProperty("CoralStation")
	var coralStation: CoralStation = KL

	override fun createSegment(): CoralStationSegment {
		return CoralStationSegment(coralStation, startingSide, endingSide, isClockwise)
	}
}
package frc.robot.autonomous.segments.models

import com.fasterxml.jackson.annotation.JsonProperty
import frc.robot.autonomous.AutonomousRoutine
import frc.robot.field.ReefSide
import frc.robot.field.ReefSide.AB

class AutonomousRoutineModel {
	@set:JsonProperty("StartingPosition")
	var startingPosition: Int = 1

	@set:JsonProperty("StartingReefSide")
	var startingReefSide: ReefSide = AB

	@set:JsonProperty("Segments")
	var segments: List<AutonomousSegmentModel> = listOf()

	fun generateRoutine(): AutonomousRoutine {
		return AutonomousRoutine(
			startingPosition,
			startingReefSide,
			List(segments.size) { index: Int -> segments[index].createSegment() },
		)
	}
}
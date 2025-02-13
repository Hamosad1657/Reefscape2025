package frc.robot.autonomous.segments.models

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.annotation.JsonSubTypes
import com.fasterxml.jackson.annotation.JsonTypeInfo
import frc.robot.autonomous.segments.AutonomousSegment
import frc.robot.field.ReefSide
import frc.robot.field.ReefSide.AB

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "Type")
@JsonSubTypes(
	JsonSubTypes.Type(value = CoralScoreSegmentModel::class, name = "CoralScoreSegment"),
	JsonSubTypes.Type(value = CoralStationSegmentModel::class, name = "CoralStationSegment"),
	JsonSubTypes.Type(value = MoveToSegmentModel::class, name = "MoveToSegment"),
	JsonSubTypes.Type(value = CollectAlgaeSegmentModel::class, name = "CollectAlgaeSegment"),
)
abstract class AutonomousSegmentModel {
	@set:JsonProperty("StartingSide")
	var startingSide: ReefSide = AB

	@set:JsonProperty("EndingSide")
	var endingSide: ReefSide = AB

	@set:JsonProperty("IsClockwise")
	var isClockwise: Boolean = false

	abstract fun createSegment(): AutonomousSegment
}
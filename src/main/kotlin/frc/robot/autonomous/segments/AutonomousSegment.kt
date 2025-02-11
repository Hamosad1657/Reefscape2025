package frc.robot.autonomous.segments

import com.fasterxml.jackson.annotation.JsonSubTypes
import com.fasterxml.jackson.annotation.JsonTypeInfo
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.field.ReefSide

/**
 * An abstract superclass representing a segment of an autonomous routine.
 *
 * @param startingSide - Side to start executing the segment's command from.
 * @param endingSide - The side in which the segment ends.
 * @param isClockwise - Whether the routine gets to [startingSide] by going clockwise around the reef or counterclockwise.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "type")
@JsonSubTypes(
	JsonSubTypes.Type(value = MoveToSegment::class, name = "MoveToSegment"),
	JsonSubTypes.Type(value = CoralScoreSegment::class, name = "CoralScoreSegment"),
	JsonSubTypes.Type(value = CoralStationSegment::class, name = "CoralStationSegment"),
	JsonSubTypes.Type(value = CollectAlgaeSegment::class, name = "CollectAlgaeSegment"),
	JsonSubTypes.Type(value = EjectAlgaeToNetSegment::class, name = "EjectAlgaeToNetSegment"),
	JsonSubTypes.Type(value = EjectAlgaeToProcessorSegment::class, name = "EjectAlgaeToProcessorSegment"),
)
abstract class AutonomousSegment(
	val startingSide: ReefSide,
	val endingSide: ReefSide,
	val isClockwise: Boolean,
) {
	abstract fun generateCommand(alliance: Alliance): Command
}
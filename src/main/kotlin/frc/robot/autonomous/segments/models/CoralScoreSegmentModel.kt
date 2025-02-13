package frc.robot.autonomous.segments.models

import com.fasterxml.jackson.annotation.JsonProperty
import frc.robot.autonomous.segments.CoralScoreSegment
import frc.robot.field.Branch
import frc.robot.field.Pipe
import frc.robot.field.Pipe.A
import frc.robot.field.PipeLevel
import frc.robot.field.PipeLevel.L1

class CoralScoreSegmentModel: AutonomousSegmentModel() {
	@set:JsonProperty("PipeToScoreOn")
	var pipeToScoreOn: Pipe = A

	@set:JsonProperty("LevelToScoreOn")
	var levelToScoreOn: PipeLevel = L1

	override fun createSegment(): CoralScoreSegment {
		return CoralScoreSegment(Branch(pipeToScoreOn, levelToScoreOn), isClockwise)
	}
}
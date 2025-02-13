package frc.robot.autonomous

import com.fasterxml.jackson.databind.ObjectMapper
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.autonomous.segments.AutonomousSegment
import frc.robot.autonomous.segments.models.AutonomousRoutineModel
import frc.robot.commands.*
import frc.robot.field.ReefSide
import frc.robot.subsystems.swerve.SwerveSubsystem
import java.io.File

/**
 * Represents an autonomous routine for the robot to follow.
 *
 * @param startingPosition - The starting position (1 -> 6) the robot starts from.
 * @param startingReefSide - The reef side to go to from the starting position.
 * @param segments - The segments to be executed in order.
 */
class AutonomousRoutine(
	private val startingPosition: Int,
	private val startingReefSide: ReefSide,
	private val segments: List<AutonomousSegment>,
) {
	fun generateCommand(alliance: Alliance): Command {
		val command = SequentialCommandGroup()

		command.addCommands(
			SwerveSubsystem.followInitialPathCommand(
				PathPlannerPath.fromPathFile("Position $startingPosition to ${startingReefSide.name}-far"),
				alliance == Alliance.Red,
			)
		)

		for (i in 0..segments.lastIndex) {
			val currentSegment = segments[i]
			// Add the driving command to the correct reef side if needed.
			if (i != 0) {
				if (segments[i-1].endingSide != currentSegment.startingSide) {
					command.addCommands(
						SwerveSubsystem.followPathCommand(
							generatePathAroundReef(
								segments[i-1].endingSide,
								currentSegment.startingSide,
								startClose = false,
								endClose = false,
								far = true,
								useQuickest = false,
								isClockwise = currentSegment.isClockwise,
							),
							alliance == Alliance.Red,
						)
					)
				}
			} else {
				if (startingReefSide != currentSegment.startingSide) {
					command.addCommands(
						SwerveSubsystem.followPathCommand(
							generatePathAroundReef(
								startingReefSide,
								currentSegment.startingSide,
								startClose = false,
								endClose = false,
								far = true,
								useQuickest = false,
								isClockwise = currentSegment.isClockwise,
							),
							alliance == Alliance.Red,
						)
					)
				}
			}

			// Add the segment's command.
			command.addCommands(currentSegment.generateCommand(alliance))
		}

		return command
	}

	companion object {
		fun createFromJSON(name: String): AutonomousRoutine {
			val objectMapper = ObjectMapper()
			val file = File(Filesystem.getDeployDirectory(), "${name}.json")

			return objectMapper.readValue(file, AutonomousRoutineModel::class.java).generateRoutine()
		}
	}
}
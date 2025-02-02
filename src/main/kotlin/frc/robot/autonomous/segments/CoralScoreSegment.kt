package frc.robot.autonomous.segments

import com.hamosad1657.lib.Branch
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.FieldConstants
import frc.robot.commands.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * A segment which gets to a pipe and scores on it.
 *
 * @param branchToScoreOn - The branch (pipe + level) the robot should score in. Note that the robot can't score on L1 autonomously.
 * @param isClockwise - Whether the command gets to the target pipe by going clockwise around the reef or counterclockwise.
// * @param far - Whether the robot should go around the reef from close or from afar. Going close is faster, but more dangerous,
// * as the robot has a harder time seeing april tags, and might hit something. Going far will be slower, but safer.
 */
class CoralScoreSegment(
	private val branchToScoreOn: Branch,
	isClockwise: Boolean,
): AutonomousSegment(branchToScoreOn.pipe.side, branchToScoreOn.pipe.side, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName("Score coral on pipe ${branchToScoreOn.pipe.letter} level ${branchToScoreOn.level.level}") {
		(ElevatorJointSubsystem.maintainElevatorJointStateCommand(
			when (branchToScoreOn.level.level) {
				2 -> ElevatorJointState.L2
				3 -> ElevatorJointState.L3
				4 -> ElevatorJointState.L4
				else -> ElevatorJointState(0.0.meters, Rotation2d()).also {
					DriverStation.reportError("Invalid coral score segment", true)
				}
			},
			true,
		) raceWith (
			// Wait for elevator to get to state
			waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
				// Align to the pipe
				SwerveSubsystem.alignToPipe(branchToScoreOn.pipe, alliance) andThen
				// Eject a coral
				(GrabberSubsystem.ejectCoralCommand() withTimeout(1.0) finallyDo {LEDsSubsystem.currentMode = ACTION_FINISHED}) andThen
				// Get back to the pose around the reef
				SwerveSubsystem.alignToPoseCommand({FieldConstants.Poses.FAR_POSES[branchToScoreOn.pipe.side.sideNumber * 2]}, true)
		)
		) andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
	}
}
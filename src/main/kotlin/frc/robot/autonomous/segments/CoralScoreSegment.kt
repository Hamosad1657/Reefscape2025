package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.rotations
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.field.FieldConstants
import frc.robot.commands.*
import frc.robot.commands.GrabberVoltageMode.*
import frc.robot.field.Branch
import frc.robot.field.PipeLevel.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
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
	override fun generateCommand(alliance: Alliance) = withName("Score coral on pipe ${branchToScoreOn.pipe.letter} level ${branchToScoreOn.level.number}") {
		(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(
			true,
			when (branchToScoreOn.level.number) {
				1 -> JointedElevatorState.L1
				2 -> JointedElevatorState.L2
				3 -> JointedElevatorState.L3
				4 -> JointedElevatorState.L4
				else -> JointedElevatorState(0.0.rotations, Rotation2d()).also {
					DriverStation.reportError("Invalid coral score segment", true)
				}
			},
		) raceWith (
	// Wait for elevator to get to state
	waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen
	// Align to the pipe
	SwerveSubsystem.alignToPipeCommand({ branchToScoreOn.pipe} , alliance) andThen
	// Eject a coral
	(GrabberSubsystem.setVoltageCommand(
		true,
		when (branchToScoreOn.level) {
			L1 -> EJECT_TO_L1
			L2, L3 -> EJECT_TO_L2_AND_L3
			L4 -> EJECT_TO_L4
		},
	) withTimeout(2.0) finallyDo {LEDsSubsystem.currentMode = ACTION_FINISHED}) andThen
	// Get back to the pose around the reef
	SwerveSubsystem.alignToPoseCommand(
		{
			val pose = FieldConstants.Poses.FAR_POSES[branchToScoreOn.pipe.side.number * 2]
			if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
		},
		true,
	))) andThen waitUntil { JointedElevatorSubsystem.isWithinTolerance }
	}
}
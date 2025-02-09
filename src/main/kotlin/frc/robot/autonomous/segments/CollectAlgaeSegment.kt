package frc.robot.autonomous.segments

import com.hamosad1657.lib.Branch
import com.hamosad1657.lib.ReefAlgae
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.FieldConstants
import frc.robot.commands.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

class CollectAlgaeSegment(
	private val algaeToCollect: ReefAlgae,
	isClockwise: Boolean,
): AutonomousSegment(algaeToCollect.reefSide, algaeToCollect.reefSide, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName("collect algae from pipe ${algaeToCollect.reefSide} level ${algaeToCollect.level.number}") {
		(ElevatorJointSubsystem.maintainElevatorJointStateCommand(
			when (algaeToCollect.level.number) {
				2 -> ElevatorJointState.LOW_REEF_ALGAE
				3 -> ElevatorJointState.HIGH_REEF_ALGAE
				else -> ElevatorJointState(0.0.meters, Rotation2d()).also {
					DriverStation.reportError("Invalid algae collect segment", true)
				}
			},
			true,
		) raceWith (
			// Wait for elevator to get to state
			waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
				// Align to the pipe
				SwerveSubsystem.alignToReefSide(algaeToCollect.reefSide, alliance) andThen
				// Eject a coral
				(GrabberSubsystem.intakeAlgaeCommand() withTimeout(2.0) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED }) andThen
				// Get back to the pose around the reef
				SwerveSubsystem.alignToPoseCommand(
					{
						val pose = FieldConstants.Poses.FAR_POSES[algaeToCollect.reefSide.number * 2]
						if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
					},
					true,
				)
			)
			) andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
	}
}
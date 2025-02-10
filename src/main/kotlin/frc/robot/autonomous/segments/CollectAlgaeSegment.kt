package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.FieldConstants
import frc.robot.commands.*
import frc.robot.field.AlgaeHeight.HIGH
import frc.robot.field.AlgaeHeight.LOW
import frc.robot.field.ReefAlgae
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

class CollectAlgaeSegment(
	private val algaeToCollect: ReefAlgae,
	isClockwise: Boolean,
): AutonomousSegment(algaeToCollect.side, algaeToCollect.side, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName(
		"Collect algae from side ${algaeToCollect.side.name} ${algaeToCollect.height.name}"
	) {
		(ElevatorJointSubsystem.maintainElevatorJointStateCommand(
			when (algaeToCollect.height) {
				LOW -> ElevatorJointState.LOW_REEF_ALGAE
				HIGH -> ElevatorJointState.HIGH_REEF_ALGAE
			},
			true,
		) raceWith (
			// Wait for elevator to get to state
			waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
				// Align to the reef side
				((SwerveSubsystem.alignToReefSide(algaeToCollect.side, alliance) andThen wait(2.0)) raceWith
				// collect algae
				GrabberSubsystem.intakeAlgaeCommand() finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED }) andThen
				// Get back to the pose around the reef
				SwerveSubsystem.alignToPoseCommand(
					{
						val pose = FieldConstants.Poses.FAR_POSES[algaeToCollect.side.number * 2]
						if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
					},
					true,
				)
			)
			) andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
	}
}
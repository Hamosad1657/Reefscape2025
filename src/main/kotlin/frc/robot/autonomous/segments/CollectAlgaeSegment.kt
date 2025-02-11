package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.field.FieldConstants
import frc.robot.commands.*
import frc.robot.field.AlgaeHeight.HIGH
import frc.robot.field.AlgaeHeight.LOW
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.field.ReefAlgae
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem

class CollectAlgaeSegment(
	private val algaeToCollect: ReefAlgae,
	isClockwise: Boolean,
): AutonomousSegment(algaeToCollect.side, algaeToCollect.side, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName(
		"Collect algae from side ${algaeToCollect.side.name} ${algaeToCollect.height}"
	) {
		(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(
			true,
			when (algaeToCollect.height) {
				LOW -> JointedElevatorState.LOW_REEF_ALGAE
				HIGH -> JointedElevatorState.HIGH_REEF_ALGAE
			},
		) raceWith (
			// Wait for elevator to get to state
			waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen
				// Align to the reef side
				((SwerveSubsystem.alignToReefSideCommand({ algaeToCollect.side }, alliance) andThen wait(1.5)) raceWith
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
			) andThen waitUntil { JointedElevatorSubsystem.isWithinTolerance }
	}
}
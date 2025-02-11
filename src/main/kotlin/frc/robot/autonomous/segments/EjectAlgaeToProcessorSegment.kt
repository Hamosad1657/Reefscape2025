package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.FieldConstants
import frc.robot.commands.*
import frc.robot.commands.GrabberEjectMode.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

class EjectAlgaeToProcessorSegment(
	startingSide: ReefSide,
	endingSide: ReefSide,
	isClockwise: Boolean,
	): AutonomousSegment(startingSide, endingSide, isClockwise) {
		override fun generateCommand(alliance: Alliance) = withName("Eject algae in processor") {
			// Get to Processor
			SwerveSubsystem.followPathCommand(
				PathPlannerPath.fromPathFile("${startingSide.sideName}-far to processor"),
				alliance == Red,
			)
			// Correct position
			SwerveSubsystem.alignToPoseCommand(
				{
					val pose = FieldConstants.Poses.AT_PROCESSOR
					if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
				},
				true,
			) andThen
			// Set elevator state
				(ElevatorJointSubsystem.maintainElevatorJointStateCommand(
				ElevatorJointState.PROCESSOR,
				true
			) raceWith(
			// Wait until elevator state is in tolerance
			waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
			// Eject in processor
			(GrabberSubsystem.ejectCommand(PROCESSOR) withTimeout(2.0) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED })) andThen
			// Get back to reef
			SwerveSubsystem.followPathCommand(
				PathPlannerPath.fromPathFile("processor to ${endingSide.sideName}-far"),
				alliance == Red,
			)
			// correct pose
			SwerveSubsystem.alignToPoseCommand(
				{
					val pose = FieldConstants.Poses.FAR_POSES[endingSide.number * 2]
					if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
				},
				true,
			) andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
		}
	}
package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.FieldConstants
import frc.robot.commands.*
import frc.robot.commands.GrabberEjectMode.*
import frc.robot.field.ReefSide
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
			// Set elevator state
			(ElevatorJointSubsystem.maintainElevatorJointStateCommand(
				ElevatorJointState.PROCESSOR,
				true,
			) raceWith(
			// Get to Processor
			SwerveSubsystem.followPathCommand(
				PathPlannerPath.fromPathFile("${startingSide.name}-far to processor"),
				alliance == Red,
			) andThen (
			// Wait until elevator state is in tolerance
			waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
			// Eject in processor
			(GrabberSubsystem.ejectCommand(PROCESSOR) withTimeout(1.5) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED })) andThen
			// Get back to reef
			SwerveSubsystem.followPathCommand(
				PathPlannerPath.fromPathFile("Processor to ${endingSide.name}-far"),
				alliance == Red,
			))) andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
		}
	}
package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.commands.*
import frc.robot.commands.GrabberEjectMode.*
import frc.robot.field.ReefSide
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

class EjectAlgaeToNetSegment(
	startingSide: ReefSide,
	endingSide: ReefSide,
	isClockwise: Boolean,
): AutonomousSegment(startingSide, endingSide, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName("Eject algae to net") {
		// Get to Net Position
		SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("${startingSide.name}-far to net position"),
			alliance == Red,
		) andThen
		// Tell elevator to be in state
		(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(
			true,
			JointedElevatorState.NET,
		) raceWith (
		// Wait until elevator state is in tolerance
		waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen
		// Eject in net
		(GrabberSubsystem.ejectCommand(NET) withTimeout(1.2) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED })
		)) andThen // TODO: Check if robot doesn't flip
		// Get back to reef
		SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("Net position to ${endingSide.name}-far"),
			alliance == Red,
		) andThen waitUntil { JointedElevatorSubsystem.isWithinTolerance }
	}
}
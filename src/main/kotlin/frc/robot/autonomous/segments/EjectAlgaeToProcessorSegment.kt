package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.commands.*
import frc.robot.commands.GrabberVoltageMode.*
import frc.robot.field.ReefSide
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
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
		(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(
			true,
			JointedElevatorState.PROCESSOR,
		) raceWith(
			// Get to Processor
			SwerveSubsystem.followPathCommand(
				PathPlannerPath.fromPathFile("${startingSide.name}-far to processor"),
				alliance == Red,
			) andThen (
				// Wait until elevator state is in tolerance
				waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen
					// Eject in processor
					(GrabberSubsystem.setVoltageCommand(true, false, EJECT_TO_PROCESSOR) withTimeout(1.5))) andThen
				// Get back to reef
				SwerveSubsystem.followPathCommand(
					PathPlannerPath.fromPathFile("Processor to ${endingSide.name}-far"),
					alliance == Red,
				))) andThen waitUntil { JointedElevatorSubsystem.isWithinTolerance }
	}
}
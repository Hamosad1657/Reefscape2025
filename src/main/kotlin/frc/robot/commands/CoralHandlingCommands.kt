package frc.robot.commands

import com.hamosad1657.lib.commands.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*
import frc.robot.subsystems.leds.LEDsSubsystem

fun intakeCoralFromGroundCommand() = withName("Intake coral from ground") {
	(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false) { JointedElevatorState.RESTING } raceWith
		(
			(if (!IntakeSubsystem.isBeamBreakInterfered) IntakeSubsystem.intakeCommand(true) else instantCommand{}) andThen
				(IntakeSubsystem.maintainAngleCommand { IntakeConstants.FEEDING_ANGLE } until { IntakeSubsystem.isWithinAngleTolerance })
			)) andThen (JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false) { JointedElevatorState.INTAKE } raceWith (
				waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen (
					IntakeSubsystem.feedToGrabberCommand() raceWith GrabberSubsystem.loadFromIntakeCommand() finallyDo {
						LEDsSubsystem.currentMode = ACTION_FINISHED
					}
				)))
}

fun intakeCoralFromCoralStationCommand() = withName("Load coral from coral station") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.CORAL_STATION) raceWith
		(waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen
			(GrabberSubsystem.loadFromCoralStationCommand() alongWith LEDsSubsystem.runOnce {
			LEDsSubsystem.currentMode = INTAKING
		})) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED }
}
package frc.robot.commands

import com.hamosad1657.lib.commands.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem

fun intakeCoralFromGroundCommand() = withName("Intake coral from ground") {
	setCoralHandlerStateCommand(CoralHandlerState.INTAKE) andThen (IntakeSubsystem.intakeCommand() alongWith GrabberSubsystem.loadFromIntakeCommand())
}

fun intakeCoralFromCoralStationCommand() = withName("Load coral from coral station") {
	setCoralHandlerStateCommand(CoralHandlerState.CORAL_STATION) andThen GrabberSubsystem.loadFromCoralStationCommand()
}
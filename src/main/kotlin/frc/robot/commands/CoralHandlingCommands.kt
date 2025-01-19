package frc.robot.commands

import com.hamosad1657.lib.commands.*
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem

fun intakeCoralCommand() = withName("Intake coral") {
	setGrabberStateCommand(GrabberState.INTAKE) andThen (IntakeSubsystem.intakeCommand() alongWith GrabberSubsystem.loadFromIntakeCommand())
}


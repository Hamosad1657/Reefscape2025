package frc.robot.commands

import com.hamosad1657.lib.commands.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.GREEN_FLASH
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.YELLOW_STATIC
import frc.robot.subsystems.leds.LEDsSubsystem

fun intakeCoralFromGroundCommand() = withName("Intake coral from ground") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.INTAKE) raceWith
		(waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
		(IntakeSubsystem.intakeCommand() raceWith  GrabberSubsystem.loadFromIntakeCommand())) finallyDo
		{ LEDsSubsystem.currentMode = GREEN_FLASH }
}

fun intakeCoralFromCoralStationCommand() = withName("Load coral from coral station") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.CORAL_STATION) raceWith
		(waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
		(GrabberSubsystem.loadFromCoralStationCommand() alongWith LEDsSubsystem.runOnce { LEDsSubsystem.currentMode = YELLOW_STATIC })) finallyDo
		{ LEDsSubsystem.currentMode = GREEN_FLASH }
}
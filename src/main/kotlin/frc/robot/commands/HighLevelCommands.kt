package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.ElevatorState.*
import frc.robot.commands.GrabberCommands.getToAngleCommand
import frc.robot.subsystems.elevator.ElevatorConstants
import frc.robot.subsystems.elevator.ElevatorSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem

class ElevatorState(elevatorHeight: Length, grabberAngle: Rotation2d) {
	val elevatorHeight: Length
	val grabberAngle: Rotation2d

	init {
		this.elevatorHeight =
			if (elevatorHeight.meters in 0.0..ElevatorConstants.MAX_HEIGHT.meters) elevatorHeight
			else {
				robotPrintError("elevator height out of bounds")
				0.meters
			}
		this.grabberAngle =
			if (grabberAngle.degrees in 0.0..grabberAngle.degrees) grabberAngle
			else {
				robotPrintError("grabber angle out of bounds")
				0.degrees
			}
	}

	companion object {
		val COLLECT = ElevatorState(0.meters, 0.0.degrees)

		// --- Different Levels ---
		val L1 = ElevatorState(0.meters, 0.0.degrees)
		val L2 = ElevatorState(0.meters, 0.0.degrees)
		val L3 = ElevatorState(0.meters, 0.0.degrees)
		val L4 = ElevatorState(0.meters, 0.0.degrees)
	}
}

fun ElevatorSubsystem.getToElevatorState(elevatorState: ElevatorState): Command = withName("get to elevator state") {
	setHeightCommand(elevatorState.elevatorHeight) raceWith GrabberSubsystem.getToAngleCommand(elevatorState.grabberAngle) until
		{isWithinTolerance && GrabberSubsystem.isWithinTolerance}
}

fun IntakeSubsystem.collect(): Command = withName("collect") {
	intakeCommand() raceWith ElevatorSubsystem.getToElevatorState(ElevatorState.COLLECT)
}
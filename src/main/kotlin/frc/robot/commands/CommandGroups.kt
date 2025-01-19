package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.elevator.ElevatorConstants
import frc.robot.subsystems.elevator.ElevatorSubsystem
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

// TODO: Find a better name
data class GrabberState(val height: Length, val angle: Rotation2d) {
	companion object {
		val L1 = GrabberState(ElevatorConstants.L1_HEIGHT, GrabberConstants.L1_ANGLE)
		val L2 = GrabberState(ElevatorConstants.L2_HEIGHT, GrabberConstants.L2_ANGLE)
		val L3 = GrabberState(ElevatorConstants.L3_HEIGHT, GrabberConstants.L3_ANGLE)
		val L4 = GrabberState(ElevatorConstants.L4_HEIGHT, GrabberConstants.L2_ANGLE)

		val CORAL_STATION = GrabberState(ElevatorConstants.CORAL_STATION_HEIGHT, GrabberConstants.CORAL_STATION_ANGLE)
		val INTAKE = GrabberState(ElevatorConstants.INTAKE_HEIGHT, GrabberConstants.INTAKING_ANGLE) // TODO: Intaking angle to intake angle
	}
}

fun setGrabberStateCommand(grabberState: GrabberState) = withName("Set grabber state") {
	ElevatorSubsystem.setHeightCommand(grabberState.height) alongWith GrabberSubsystem.setAngleCommand(grabberState.angle)
}
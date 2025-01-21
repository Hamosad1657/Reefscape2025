package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.elevator.ElevatorConstants
import frc.robot.subsystems.elevator.ElevatorSubsystem
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

/** Represents a state of the elevator and the grabber. */
data class CoralHandlerState(val elevatorHeight: Length, val grabberAngle: Rotation2d) {
	companion object {
		val L1 = CoralHandlerState(ElevatorConstants.L1_HEIGHT, GrabberConstants.L1_ANGLE)
		val L2 = CoralHandlerState(ElevatorConstants.L2_HEIGHT, GrabberConstants.L2_ANGLE)
		val L3 = CoralHandlerState(ElevatorConstants.L3_HEIGHT, GrabberConstants.L3_ANGLE)
		val L4 = CoralHandlerState(ElevatorConstants.L4_HEIGHT, GrabberConstants.L4_ANGLE)

		val CORAL_STATION = CoralHandlerState(ElevatorConstants.CORAL_STATION_HEIGHT, GrabberConstants.CORAL_STATION_ANGLE)
		val INTAKE = CoralHandlerState(ElevatorConstants.INTAKE_HEIGHT, GrabberConstants.INTAKE_ANGLE) // TODO: Intaking angle to intake angle
	}
}

/** Sets the state of the elevator and the grabber. */
fun setCoralHandlerStateCommand(grabberState: CoralHandlerState) = withName("Set grabber state") {
	ElevatorSubsystem.setHeightCommand(grabberState.elevatorHeight) alongWith GrabberSubsystem.setAngleCommand(grabberState.grabberAngle)
}
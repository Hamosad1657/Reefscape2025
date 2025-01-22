package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.elevator.ElevatorConstants
import frc.robot.subsystems.elevator.ElevatorSubsystem
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.BLUE_FLASH
import frc.robot.subsystems.leds.LEDsSubsystem

/** Represents a state of the elevator and the grabber. */
data class CoralHandlerState(val elevatorHeight: Length, val grabberAngle: Rotation2d) {
	companion object {
		val L1 = CoralHandlerState(ElevatorConstants.L1_HEIGHT, GrabberConstants.L1_ANGLE)
		val L2 = CoralHandlerState(ElevatorConstants.L2_HEIGHT, GrabberConstants.L2_ANGLE)
		val L3 = CoralHandlerState(ElevatorConstants.L3_HEIGHT, GrabberConstants.L3_ANGLE)
		val L4 = CoralHandlerState(ElevatorConstants.L4_HEIGHT, GrabberConstants.L4_ANGLE)

		val CORAL_STATION = CoralHandlerState(ElevatorConstants.CORAL_STATION_HEIGHT, GrabberConstants.CORAL_STATION_ANGLE)
		val INTAKE = CoralHandlerState(ElevatorConstants.INTAKE_HEIGHT, GrabberConstants.INTAKE_ANGLE)
	}
}

/** Sets the state of the elevator and the grabber. */
fun setCoralHandlerStateCommand(grabberState: CoralHandlerState, useLEDs: Boolean) = withName("Set grabber state") {
	ElevatorSubsystem.setHeightCommand(grabberState.elevatorHeight) alongWith GrabberSubsystem.setAngleCommand(grabberState.grabberAngle) finallyDo
		{ if (useLEDs) LEDsSubsystem.currentMode = BLUE_FLASH }
}
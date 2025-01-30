package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.elevator.joint.ElevatorJointConstants
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.LOADING_FROM_CORAL_STATION
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.REACHED_SETPOINT
import frc.robot.subsystems.leds.LEDsSubsystem

/** Represents a state of the elevator and the grabber. */
data class ElevatorJointState(val height: Length, val angle: Rotation2d) {
	companion object {
		val L1 = ElevatorJointState(ElevatorJointConstants.L1_HEIGHT, ElevatorJointConstants.L1_ANGLE)
		val L2 = ElevatorJointState(ElevatorJointConstants.L2_HEIGHT, ElevatorJointConstants.L2_ANGLE)
		val L3 = ElevatorJointState(ElevatorJointConstants.L3_HEIGHT, ElevatorJointConstants.L3_ANGLE)
		val L4 = ElevatorJointState(ElevatorJointConstants.L4_HEIGHT, ElevatorJointConstants.L4_ANGLE)

		val LOW_REEF_ALGAE = ElevatorJointState(ElevatorJointConstants.LOW_REEF_ALGAE_HEIGHT, ElevatorJointConstants.REEF_ALGAE_ANGLE)
		val HIGH_REEF_ALGAE = ElevatorJointState(ElevatorJointConstants.HIGH_REEF_ALGAE_HEIGHT, ElevatorJointConstants.REEF_ALGAE_ANGLE)
		val PROCESSOR = ElevatorJointState(ElevatorJointConstants.PROCESSOR_HEIGHT, ElevatorJointConstants.PROCESSOR_ANGLE)
		val NET = ElevatorJointState(ElevatorJointConstants.NET_HEIGHT, ElevatorJointConstants.NET_ANGLE)

		val CORAL_STATION = ElevatorJointState(ElevatorJointConstants.CORAL_STATION_HEIGHT, ElevatorJointConstants.CORAL_STATION_ANGLE)
		val INTAKE = ElevatorJointState(ElevatorJointConstants.INTAKE_HEIGHT, ElevatorJointConstants.INTAKE_ANGLE)
	}
}

/** Maintains an elevator joint state. Never ends. */
fun ElevatorJointSubsystem.maintainElevatorJointStateCommand(state: ElevatorJointState, useLEDs: Boolean) = withName("Maintain elevator joint state") {
	run {
		setHeight(state.height)
		updateAngleControl(state.angle)
	} finallyDo {
		if (useLEDs){ LEDsSubsystem.runOnce { LEDsSubsystem.currentMode = REACHED_SETPOINT } }
	}
}

/** Maintains an elevator joint state. Never ends. */
fun ElevatorJointSubsystem.maintainElevatorJointStateCommand(state: () -> ElevatorJointState, useLEDs: Boolean) = withName("Maintain elevator joint state") {
	run {
		setHeight(state().height)
		updateAngleControl(state().angle)
	} finallyDo {
		if (useLEDs){
			LEDsSubsystem.runOnce { LEDsSubsystem.currentMode = REACHED_SETPOINT }
		}
	}
}

// --- Testing ---

fun ElevatorJointSubsystem.test_elevatorMotorsSetVoltageCommand(voltage: Volts): Command = withName("Set elevator motors voltage") {
	run { setElevatorMotorsVoltage(voltage) }
}

fun ElevatorJointSubsystem.test_angleMotorSetVoltageCommand(voltage: Volts) = withName("Set angle motor voltage") {
	run { setAngleMotorVoltage(voltage) }
}
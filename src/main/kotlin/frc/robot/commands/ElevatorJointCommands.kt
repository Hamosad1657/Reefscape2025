package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.MaintainElevatorJointStateState.*
import frc.robot.subsystems.jointedElevator.JointedElevatorConstants
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.REACHED_SETPOINT
import frc.robot.subsystems.leds.LEDsSubsystem

/** Represents a state of the elevator and the grabber. */
data class JointedElevatorState(val height: Length, val angle: Rotation2d) {
	companion object {
		val L1 = JointedElevatorState(JointedElevatorConstants.L1_HEIGHT, JointedElevatorConstants.L1_ANGLE)
		val L2 = JointedElevatorState(JointedElevatorConstants.L2_HEIGHT, JointedElevatorConstants.L2_ANGLE)
		val L3 = JointedElevatorState(JointedElevatorConstants.L3_HEIGHT, JointedElevatorConstants.L3_ANGLE)
		val L4 = JointedElevatorState(JointedElevatorConstants.L4_HEIGHT, JointedElevatorConstants.L4_ANGLE)

		val LOW_REEF_ALGAE = JointedElevatorState(JointedElevatorConstants.LOW_REEF_ALGAE_HEIGHT, JointedElevatorConstants.REEF_ALGAE_ANGLE)
		val HIGH_REEF_ALGAE = JointedElevatorState(JointedElevatorConstants.HIGH_REEF_ALGAE_HEIGHT, JointedElevatorConstants.REEF_ALGAE_ANGLE)
		val PROCESSOR = JointedElevatorState(JointedElevatorConstants.PROCESSOR_HEIGHT, JointedElevatorConstants.PROCESSOR_ANGLE)
		val NET = JointedElevatorState(JointedElevatorConstants.NET_HEIGHT, JointedElevatorConstants.NET_ANGLE)

		val CORAL_STATION = JointedElevatorState(JointedElevatorConstants.CORAL_STATION_HEIGHT, JointedElevatorConstants.CORAL_STATION_ANGLE)
		val INTAKE = JointedElevatorState(JointedElevatorConstants.INTAKE_HEIGHT, JointedElevatorConstants.INTAKE_ANGLE)
	}
}

private enum class MaintainElevatorJointStateState(val shouldExitState: () -> Boolean) {
	UP_RIGHTING({
		JointedElevatorSubsystem.isWithinAngleTolerance
	}),
	GETTING_TO_HEIGHT({
		JointedElevatorSubsystem.isWithinHeightTolerance
	}),
	GETTING_TO_ANGLE({
		JointedElevatorSubsystem.isWithinAngleTolerance
	}),
	MAINTAINING_STATE({
		false
	}),
}

/** Maintains an elevator joint state. Does not end automatically. */
fun JointedElevatorSubsystem.maintainJointedElevatorStateCommand(useLEDs: Boolean ,state: JointedElevatorState) = withName("Maintain elevator joint state") {
	maintainJointedElevatorStateCommand(useLEDs) { state }
}

/** Maintains an elevator joint state. Does not end automatically. */
fun JointedElevatorSubsystem.maintainJointedElevatorStateCommand(useLEDs: Boolean, state: () -> JointedElevatorState) = withName("Maintain elevator joint state") {
	var currentState = UP_RIGHTING
	runOnce { currentState = UP_RIGHTING; isMaintainingState = false } andThen run {
		when (currentState) {
			UP_RIGHTING -> {
				updateAngleControl(JointedElevatorConstants.INTAKE_ANGLE)

				if (currentState.shouldExitState()) {
					currentState = GETTING_TO_HEIGHT
				}
			}
			GETTING_TO_HEIGHT -> {
				updateAngleControl(JointedElevatorConstants.INTAKE_ANGLE)
				setHeight(state().height)

				if (currentState.shouldExitState()) {
					currentState = GETTING_TO_ANGLE
				}
			}
			GETTING_TO_ANGLE -> {
				updateAngleControl(state().angle)
				setHeight(state().height)

				if (currentState.shouldExitState()) {
					currentState = MAINTAINING_STATE
					if (useLEDs) LEDsSubsystem.currentMode = REACHED_SETPOINT
					isMaintainingState = true
				}
			}
			MAINTAINING_STATE -> {
				updateAngleControl(state().angle)
				setHeight(state().height)
			}
		}
	}
}

// --- Testing ---

fun JointedElevatorSubsystem.test_elevatorMotorsSetVoltageCommand(voltage: Volts): Command = withName("Set elevator motors voltage") {
	run { setElevatorMotorsVoltage(voltage) }
}

fun JointedElevatorSubsystem.test_angleMotorSetVoltageCommand(voltage: Volts) = withName("Set angle motor voltage") {
	run { setAngleMotorVoltage(voltage) }
}
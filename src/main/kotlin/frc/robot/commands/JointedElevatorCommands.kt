package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.MaintainElevatorJointStateState.*
import frc.robot.subsystems.jointedElevator.JointedElevatorConstants
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem

fun JointedElevatorSubsystem.maintainGrabberAngleCommand(angle: () -> Rotation2d) = withName("Maintain grabber angle") {
	run { updateAngleControl(angle()) }
}

fun JointedElevatorSubsystem.maintainElevatorRotationCommand(rotation: () -> Rotation2d) = withName("Maintain elevator rotation") {
	run { updateElevatorRotationControl(rotation()) }
}

/** Represents a state of the elevator and the grabber. */
data class JointedElevatorState(val elevatorRotation: Rotation2d, val angle: Rotation2d) {
	companion object {
		val RESTING = JointedElevatorState(JointedElevatorConstants.RESTING_ELEVATOR_ROTATION, JointedElevatorConstants.RESTING_ANGLE)

		val L1 = JointedElevatorState(JointedElevatorConstants.L1_ELEVATOR_ROTATION, JointedElevatorConstants.L1_ANGLE)
		val L2 = JointedElevatorState(JointedElevatorConstants.L2_ELEVATOR_ROTATION, JointedElevatorConstants.L2_ANGLE)
		val L3 = JointedElevatorState(JointedElevatorConstants.L3_ELEVATOR_ROTATION, JointedElevatorConstants.L3_ANGLE)
		val L4 = JointedElevatorState(JointedElevatorConstants.L4_ELEVATOR_ROTATION, JointedElevatorConstants.L4_ANGLE)

		val LOW_REEF_ALGAE = JointedElevatorState(JointedElevatorConstants.LOW_REEF_ALGAE_ELEVATOR_ROTATION, JointedElevatorConstants.REEF_ALGAE_ANGLE)
		val HIGH_REEF_ALGAE = JointedElevatorState(JointedElevatorConstants.HIGH_REEF_ALGAE_ELEVATOR_ROTATION, JointedElevatorConstants.REEF_ALGAE_ANGLE)
		val PROCESSOR = JointedElevatorState(JointedElevatorConstants.PROCESSOR_ELEVATOR_ROTATION, JointedElevatorConstants.PROCESSOR_ANGLE)
		val NET = JointedElevatorState(JointedElevatorConstants.NET_ELEVATOR_ROTATION, JointedElevatorConstants.NET_ANGLE)

		val CORAL_STATION = JointedElevatorState(JointedElevatorConstants.CORAL_STATION_ELEVATOR_ROTATION, JointedElevatorConstants.CORAL_STATION_ANGLE)
		val INTAKE = JointedElevatorState(JointedElevatorConstants.INTAKE_ELEVATOR_ROTATION, JointedElevatorConstants.INTAKE_ANGLE)
	}
}

private enum class MaintainElevatorJointStateState(val shouldExitState: () -> Boolean) {
	UP_RIGHTING({
		JointedElevatorSubsystem.isWithinAngleTolerance
	}),
	GETTING_TO_HEIGHT({
		JointedElevatorSubsystem.isElevatorWithinTolerance
	}),
	GETTING_TO_ANGLE({
		JointedElevatorSubsystem.isWithinAngleTolerance
	}),
	MAINTAINING_STATE({
		false
	}),
}

/** Maintains an elevator joint state. Does not end automatically. */
fun JointedElevatorSubsystem.maintainJointedElevatorStateCommand(state: JointedElevatorState) = withName("Maintain elevator joint state") {
	maintainJointedElevatorStateCommand({ state })
}

/** Maintains an elevator joint state. Does not end automatically. */
fun JointedElevatorSubsystem.maintainJointedElevatorStateCommand(state: () -> JointedElevatorState) = withName("Maintain elevator joint state") {
	var currentState = UP_RIGHTING
	runOnce { currentState = UP_RIGHTING; isMaintainingState = false } andThen run {
		when (currentState) {
			UP_RIGHTING -> {
				updateAngleControl(JointedElevatorConstants.RESTING_ANGLE)
				updateElevatorRotationControl()
				if (currentState.shouldExitState()) {
					currentState = GETTING_TO_HEIGHT
				}
			}
			GETTING_TO_HEIGHT -> {
				updateAngleControl(JointedElevatorConstants.RESTING_ANGLE)
				updateElevatorRotationControl(state().elevatorRotation)
				if (currentState.shouldExitState()) {
					currentState = GETTING_TO_ANGLE
				}
			}
			GETTING_TO_ANGLE -> {
				updateAngleControl(state().angle)
				updateElevatorRotationControl(state().elevatorRotation)
				if (currentState.shouldExitState()) {
					currentState = MAINTAINING_STATE
					isMaintainingState = true
				}
			}
			MAINTAINING_STATE -> {
				updateAngleControl(state().angle)
				updateElevatorRotationControl(state().elevatorRotation)
			}
		}
	}
}

// --- Testing ---

fun JointedElevatorSubsystem.test_elevatorMotorsSetVoltageCommand(voltage: () -> Volts): Command = withName("Set elevator motors voltage") {
	run { setElevatorMotorsVoltage(voltage()) }
}

fun JointedElevatorSubsystem.test_angleMotorSetVoltageCommand(voltage: () -> Volts) = withName("Set angle motor voltage") {
	run { setAngleMotorVoltage(voltage()) }
}
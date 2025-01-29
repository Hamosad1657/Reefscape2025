   package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import frc.robot.commands.LoadFromIntakeState.*
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

/** Runs the grabber motor in a way that intakes a coral through it's back and ejects it through it's front. Doesn't end automatically. */
fun GrabberSubsystem.ejectCoralCommand() = withName("Run inwards") {
	run { setMotorVoltage(GrabberConstants.CORAL_FORWARD_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

/** Runs the grabber motor in a way that ejects a coral through it's back and intakes it through it's front. Doesn't end automatically. */
fun GrabberSubsystem.loadCoralCommand() = withName("Run outwards") {
	run { setMotorVoltage(GrabberConstants.CORAL_BACKWARD_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

/** Runs the grabber motor so that it will eject an algae to go into the processor. Does not end automatically. */
fun GrabberSubsystem.ejectAlgaeToProcessorCommand() = withName("Eject Algae to processor") {
	run { setMotorVoltage(GrabberConstants.EJECT_ALGAE_TO_PROCESSOR_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

/** Runs the grabber motor so that it will eject an algae to go into the net. Does not end automatically. */
fun GrabberSubsystem.ejectAlgaeToNetCommand() = withName("Eject algae to net") {
	run { setMotorVoltage(GrabberConstants.EJECT_ALGAE_TO_NET_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

/** Runs the motor inwards to intake an algae. Ends when the beam break is interfered. */
fun GrabberSubsystem.intakeAlgaeCommand() = withName("Intake Algae") {
	run { setMotorVoltage(GrabberConstants.INTAKE_ALGAE_VOLTAGE) } until { isBeamBreakInterfered } finallyDo {
		stopMotor()
	}
}

enum class LoadFromIntakeState(val shouldExitState: () -> Boolean) {
	Intaking(shouldExitState = {
		GrabberSubsystem.isBeamBreakInterfered
	}),
	Loading(shouldExitState = {
		!GrabberSubsystem.isBeamBreakInterfered
	}),
	Holding(shouldExitState = {
		GrabberSubsystem.isInTolerance
	}),
	Finished(shouldExitState = {
		false
	})
}

fun GrabberSubsystem.loadFromIntakeCommand() = withName("Load from intake command") {
	var loadFromIntakeState: LoadFromIntakeState = Intaking
	run {
		when (loadFromIntakeState) {
			Intaking -> {
				setMotorVoltage(GrabberConstants.CORAL_FORWARD_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Loading
			}
			Loading -> {
				setMotorVoltage(GrabberConstants.CORAL_FORWARD_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) {
					loadFromIntakeState = Holding
					setMotorSetpoint(Length.fromCentimeters(-3))
				}
			}
			Holding -> {
				updateMotorPIDControl()
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Finished
			}
			Finished -> {}
		}
	} until { loadFromIntakeState == Finished } finallyDo { stopMotor() }
}

enum class LoadFromCoralStationState(val shouldExitState: () -> Boolean) {
	Loading(shouldExitState = {
		GrabberSubsystem.isBeamBreakInterfered
	}),
	Holding(shouldExitState = {
		GrabberSubsystem.isInTolerance
	}),
	Finished(shouldExitState = {
		false
	})
}

fun GrabberSubsystem.loadFromCoralStationCommand() = withName("Load from coral station") {
	var loadFromCoralStationState = LoadFromCoralStationState.Loading
	run {
		when (loadFromCoralStationState) {
			LoadFromCoralStationState.Loading -> {
				setMotorVoltage(GrabberConstants.CORAL_BACKWARD_VOLTAGE)
				if (loadFromCoralStationState.shouldExitState()) {
					loadFromCoralStationState = LoadFromCoralStationState.Holding
					setMotorSetpoint(Length.fromCentimeters(-3))
				}
			}
			LoadFromCoralStationState.Holding -> {
				updateMotorPIDControl()
				if (loadFromCoralStationState.shouldExitState()) loadFromCoralStationState = LoadFromCoralStationState.Finished
			}
			LoadFromCoralStationState.Finished -> {}
		}
	} until { loadFromCoralStationState == LoadFromCoralStationState.Finished } finallyDo { stopMotor() }
}

//--- Test commands ---

fun GrabberSubsystem.test_setWheelsVoltageCommand(voltage: Volts) = withName("Set wheels voltage") {
	run { setMotorVoltage(voltage) }
}
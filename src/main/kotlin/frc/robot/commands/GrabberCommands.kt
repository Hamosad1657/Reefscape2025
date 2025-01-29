package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rotations
import frc.robot.commands.LoadFromIntakeState.*
import frc.robot.commands.LoadFromIntakeState.Finished
import frc.robot.commands.LoadFromIntakeState.Holding
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

/** Runs the grabber motor in a way that intakes a coral through it's back and ejects it through it's front. */
fun GrabberSubsystem.runForwardsCommand() = withName("Run inwards") {
	run { setMotorVoltage(GrabberConstants.CORAL_FORWARDS_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

/** Runs the grabber motor in a way that ejects a coral through it's back and intakes it through it's front. */
fun GrabberSubsystem.runBackwardsCommand() = withName("Run outwards") {
	run { setMotorVoltage(GrabberConstants.CORAL_BACKWARDS_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

fun GrabberSubsystem.intakeAlgaeCommand() = withName("Grab algae") {
	run { setMotorVoltage(GrabberConstants.INTAKE_ALGAE_VOLTAGE) } until { isBeamBreakInterfered } finallyDo {
		stopMotor()
	}
}

fun GrabberSubsystem.ejectAlgaeCommand(voltage: Volts) = withName("Eject Algae") {
	run { setMotorVoltage(voltage) } withTimeout GrabberConstants.ALGAE_EJECT_TIMEOUT_SEC finallyDo {
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
		((-3 / GrabberConstants.LENGTH_FOR_EACH_ROTATION.asCentimeters).rotations - GrabberSubsystem.currentAngle).rotations  < GrabberConstants.MOTOR_TOLERANCE.rotations
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
				setMotorVoltage(GrabberConstants.CORAL_FORWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Loading
			}
			Loading -> {
				setMotorVoltage(GrabberConstants.CORAL_FORWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) {
					loadFromIntakeState = Holding
					setMotorSetpoint(Length.fromCentimeters(-3))
				}
			}
			Holding -> {
				updateMotorPIDControl()
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Finished
			}
			Finished -> {
				stopMotor()
			}
		}
	} until { loadFromIntakeState == Finished }
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
				setMotorVoltage(GrabberConstants.CORAL_BACKWARDS_VOLTAGE)
				if (loadFromCoralStationState.shouldExitState()) {
					loadFromCoralStationState = LoadFromCoralStationState.Holding
					setMotorSetpoint(Length.fromCentimeters(-3))
				}
			}
			LoadFromCoralStationState.Holding -> {
				updateMotorPIDControl()
				if (loadFromCoralStationState.shouldExitState()) loadFromCoralStationState = LoadFromCoralStationState.Finished
			}
			LoadFromCoralStationState.Finished -> {
				stopMotor()
			}
		}
	} until { loadFromCoralStationState == LoadFromCoralStationState.Finished }
}

//--- Test commands ---

fun GrabberSubsystem.test_setWheelsVoltageCommand(voltage: Volts) = withName("Set wheels voltage") {
	run { setMotorVoltage(voltage) }
}
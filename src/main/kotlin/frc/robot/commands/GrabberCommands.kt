package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import frc.robot.commands.LoadFromIntakeState.*
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

/** Runs the grabber motor in a way that intakes a coral through it's back and ejects it through it's front. */
fun GrabberSubsystem.runForwardsCommand() = withName("Run inwards") {
	run { setMotorVoltage(GrabberConstants.WHEELS_FORWARDS_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

/** Runs the grabber motor in a way that ejects a coral through it's back and intakes it through it's front. */
fun GrabberSubsystem.runBackwardsCommand() = withName("Run outwards") {
	run { setMotorVoltage(GrabberConstants.WHEELS_BACKWARDS_VOLTAGE) } finallyDo {
		stopMotor()
	}
}

enum class LoadFromIntakeState(val shouldExitState: () -> Boolean) {
	Intaking(shouldExitState = {
		GrabberSubsystem.isCoralInBeamBreak
	}),
	Loading(shouldExitState = {
		!GrabberSubsystem.isCoralInBeamBreak
	}),
	Holding(shouldExitState = {
		GrabberSubsystem.isCoralInBeamBreak
	}),
	Finished(shouldExitState = {
		false
	})
}

fun GrabberSubsystem.loadFromIntakeCommand() = withName("Load from intake command") {
	var loadFromIntakeState = Intaking
	run {
		when (loadFromIntakeState) {
			Intaking -> {
				setMotorVoltage(GrabberConstants.WHEELS_FORWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Loading
			}
			Loading -> {
				setMotorVoltage(GrabberConstants.WHEELS_FORWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Holding
			}
			Holding -> {
				setMotorVoltage(GrabberConstants.WHEELS_BACKWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Finished
			}
			Finished -> {
				stopMotor()
			}
		}
	} until { loadFromIntakeState == Finished }
}

fun GrabberSubsystem.loadFromCoralStationCommand() = withName("Load from coral station") {
	run { setMotorVoltage(GrabberConstants.WHEELS_BACKWARDS_VOLTAGE) } until { isCoralInBeamBreak }
}

//--- Test commands ---

fun GrabberSubsystem.test_setWheelsVoltageCommand(voltage: Volts) = withName("Set wheels voltage") {
	run { setMotorVoltage(voltage) }
}
package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.commands.LoadFromIntakeState.*
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

/** Runs the grabber motor in a way that intakes a coral from it's back and ejects it from it's front. */
fun GrabberSubsystem.runForwardsCommand() = withName("Run inwards") {
	run { setWheelsMotorVoltage(GrabberConstants.WHEELS_FORWARDS_VOLTAGE) } finallyDo {
		stopWheelsMotor()
	}
}

/** Runs the grabber motor in a way that ejects a coral from it's back and intakes it from it's front. */
fun GrabberSubsystem.runBackwardsCommand() = withName("Run outwards") {
	run { setWheelsMotorVoltage(GrabberConstants.WHEELS_BACKWARDS_VOLTAGE) } finallyDo {
		stopWheelsMotor()
	}
}

fun GrabberSubsystem.setAngleCommand(angle: Rotation2d) = withName("Get to angle") {
	runOnce{ angleSetpoint = angle }
}

enum class LoadFromIntakeState(val shouldExitState: () -> Boolean) {
	PreLoading(shouldExitState = {
		GrabberSubsystem.isCoralInBeamBreak
	}),
	Loading(shouldExitState = {
		!GrabberSubsystem.isCoralInBeamBreak
	}),
	Hold(shouldExitState = {
		GrabberSubsystem.isCoralInBeamBreak
	}),
	Finished(shouldExitState = {
		false
	})
}

fun GrabberSubsystem.loadFromIntakeCommand() = withName("Load from intake command") {
	var loadFromIntakeState = PreLoading
	run {
		when (loadFromIntakeState) {
			PreLoading -> {
				setWheelsMotorVoltage(GrabberConstants.WHEELS_FORWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Loading
			}
			Loading -> {
				setWheelsMotorVoltage(GrabberConstants.WHEELS_FORWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Hold
			}
			Hold -> {
				setWheelsMotorVoltage(GrabberConstants.WHEELS_BACKWARDS_VOLTAGE)
				if (loadFromIntakeState.shouldExitState()) loadFromIntakeState = Finished
			}
			Finished -> {
				stopWheelsMotor()
			}
		}
	} until { loadFromIntakeState == Finished }
}

fun GrabberSubsystem.loadFromCoralStationCommand() {
	run { setWheelsMotorVoltage(GrabberConstants.WHEELS_BACKWARDS_VOLTAGE) } until { isCoralInBeamBreak }
}

//--- Test commands ---

fun GrabberSubsystem.test_setWheelsVoltageCommand(voltage: Volts) = withName("Set wheels voltage") {
	run { setWheelsMotorVoltage(voltage) }
}

fun GrabberSubsystem.test_setAngleMotorVoltageCommand(voltage: Volts) = withName("Set angle motor voltage") {
	run { setAngleMotorVoltage(voltage) }
}
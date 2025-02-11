package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import frc.robot.commands.IntakeState.*
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*
import frc.robot.subsystems.leds.LEDsSubsystem

// --- Wheels commands ---

/** Runs the wheel motor so that it will intake a coral and drive it towards the elevator. Doesn't end automatically. */
fun IntakeSubsystem.runWheelMotorCommand() = withName("Run motor") {
	run { runWheelMotor() } finallyDo { stopWheelMotor() }
}

/** Runs the wheel motor so that it will eject a coral. Doesn't end automatically. */
fun IntakeSubsystem.runWheelMotorReverseCommand() = withName("Run motor reverse") {
	run { runWheelMotorReverse() } finallyDo { stopWheelMotor() }
}

/** Stops the wheel motor. Ends instantly. */
fun IntakeSubsystem.stopWheelMotorCommand() = withName("Stop motor") {
	runOnce { stopWheelMotor() }
}

// --- Angle commands ---

/** Maintains a retracted angle. Does not end automatically. */
fun IntakeSubsystem.retractIntakeCommand() = withName("Retract intake") {
	run { setAngleToRetracted() }
}

/** Sets the intake angle to the deployed angle. Ends instantly. */
fun IntakeSubsystem.deployIntakeCommand() = withName("Deploy intake") {
	run { setAngleToDeploy() }
}

// --- Intake command ---

private enum class IntakeState(val shouldExitState: () -> Boolean) {
	Deploying(shouldExitState = {
		IntakeSubsystem.isWithinAngleTolerance
	}),
	Intaking(shouldExitState = {
		IntakeSubsystem.isBeamBreakInterfered
	}),
	Retracting(shouldExitState = {
		false
	}),
}

/** Intakes from ground, does not end automatically. */
fun IntakeSubsystem.intakeCommand(useLEDs: Boolean) = withName("Intake") {
	var intakeState = Deploying
	run {
		when (intakeState) {
			Deploying -> {
				setAngleToDeploy()
				if (intakeState.shouldExitState()) {
					intakeState = Intaking
					if (useLEDs) {
						LEDsSubsystem.currentMode = REACHED_SETPOINT
					}
				}
			}
			Intaking -> {
				setAngleToDeploy()
				runWheelMotor()
				if (intakeState.shouldExitState()) intakeState = Retracting
			}
			Retracting -> {
				if (useLEDs) {
					LEDsSubsystem.currentMode = REACHED_SETPOINT
				}
				setAngleToRetracted()
				runWheelMotor()
			}
		}
	} finallyDo {
		stopWheelMotor()
		LEDsSubsystem.currentMode = ACTION_FINISHED
	}
}

// --- Testing commands ---

/** Use for testing. */
fun IntakeSubsystem.test_openLoopRunWheelsCommand(voltage: Volts) = withName("Open loop run wheels") {
	run { setWheelMotorVoltage(voltage) }
}

/** Use for testing. */
fun IntakeSubsystem.test_openLoopRunAngleControlCommand(voltage: Volts) = withName("Open loop angle control") {
	run { setAngleMotorVoltage(voltage) }
}

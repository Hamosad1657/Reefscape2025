package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import frc.robot.commands.IntakeState.*
import frc.robot.subsystems.intake.IntakeSubsystem

// --- Wheels commands ---

/** Runs the wheel motor so that it will intake a coral and/or drive it towards the elevator. Doesn't end automatically. */
fun IntakeSubsystem.runWheelMotorCommand() = withName("Run motor") {
	run { runWheelMotor() } finallyDo { stopWheelMotor() }
}

/** Runs the wheel motor so that it will move a coral in it away from the elevator. Doesn't end automatically. */
fun IntakeSubsystem.runWheelMotorReverseCommand() = withName("Run motor reverse") {
	run { runWheelMotorReverse() }
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
		IntakeSubsystem.isMotorCurrentAboveThreshold
	}),
	Retracting(shouldExitState = {
		false
	}),
}

/** Intakes from ground, does not end automatically. */
fun IntakeSubsystem.intakeCommand() = withName("Intake") {
	var intakeState = Deploying
	run {
		when (intakeState) {
			Deploying -> {
				setAngleToDeploy()
				if (intakeState.shouldExitState()) intakeState = Intaking
			}
			Intaking -> {
				setAngleToDeploy()
				runWheelMotor()
				if (intakeState.shouldExitState()) intakeState = Retracting
			}
			Retracting -> {
				setAngleToRetracted()
				runWheelMotor()
			}
		}
	} finallyDo { stopWheelMotor() }
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

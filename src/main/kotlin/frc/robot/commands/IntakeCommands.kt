package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import frc.robot.commands.IntakeState.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem

// --- Wheels commands ---

/** Runs the intake motor so that it will intake a coral and/or drive it towards the elevator. */
fun IntakeSubsystem.runMotorCommand() = withName("Run motor") {
	run { runMotor() }
}

/** Runs the intake motor in reverse so that it will move a coral in it away from the elevator. */
fun IntakeSubsystem.runMotorReverseCommand() = withName("Run motor reverse") {
	run { runMotorReverse() }
}

fun IntakeSubsystem.stopMotorCommand() = withName("Stop motor") {
	runOnce { stopMotor() }
}

// --- Angle commands ---

/** Sets the intake angle to the retracted angle. Ends instantly */
fun IntakeSubsystem.retractIntakeCommand() = withName("Retract intake") {
	runOnce { setAngleToRetract() }
}

/** Sets the intake angle to the deployed angle. Ends instantly. */
fun IntakeSubsystem.deployIntakeCommand() = withName("Deploy intake") {
	runOnce { setAngleToDeploy() }
}

private enum class IntakeState(val shouldExitState: () -> Boolean) {
	Deploying(shouldExitState = {
		IntakeSubsystem.isWithinAngleTolerance
	}),
	Intaking(shouldExitState = {
		IntakeSubsystem.isMotorCurrentAboveThreshold
	}),
	Retracting(shouldExitState = {
		GrabberSubsystem.isCoralInBeamBreak
	}),
	Finished(shouldExitState = {
		false
	}),
}

fun IntakeSubsystem.intakeCommand() = withName("Intake") {
	var intakeState = Deploying
	run {
		when (intakeState) {
			Deploying -> {
				setAngleToDeploy()
				if (intakeState.shouldExitState()) intakeState = Intaking
			}
			Intaking -> {
				runMotor()
				if (intakeState.shouldExitState()) intakeState = Retracting
			}
			Retracting -> {
				setAngleToRetract()
				runMotor()
				if (intakeState.shouldExitState()) intakeState = Finished
			}
			Finished -> {
				stopMotor()
			}
		}
	} until { intakeState == Finished }
}

// --- Testing commands ---

/** Use for testing */
fun IntakeSubsystem.test_openLoopRunWheelsCommand(voltage: Volts) = withName("Open loop run wheels") {
	run { setWheelMotorVoltage(voltage) }
}

/** Use for testing */
fun IntakeSubsystem.test_openLoopRunAngleControlCommand(voltage: Volts) = withName("Open loop angle control") {
	run { setAngleMotorVoltage(voltage) }
}

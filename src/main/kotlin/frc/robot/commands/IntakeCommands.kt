package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import frc.robot.commands.IntakeState.*
import frc.robot.subsystems.intake.IntakeSubsystem

// --- Testing commands ---

/** Use for testing */
fun IntakeSubsystem.openLoopRunWheelsCommand(voltage: Volts) = withName("Open loop run wheels") {
	run { setWheelMotorVoltage(voltage) }
}

/** Use for testing */
fun IntakeSubsystem.openLoopRunAngleControlCommand(voltage: Volts) = withName("Open loop angle control") {
	run { setAngleMotorVoltage(voltage) }
}

// --- Wheels commands ---

/** Runs the intaking motor so that it will intake a coral and/or drive it towards the elevator. ends instantly. */
fun IntakeSubsystem.runMotorCommand() = withName("Run motor") {
	runOnce { runMotor() }
}

/** Runs the intaking motor in reverse so that it will move a coral in it away from the elevator. */
fun IntakeSubsystem.runMotorReverseCommand() = withName("Run motor reverse") {
	runOnce { runMotorReverse() }
}

fun IntakeSubsystem.stopMotorCommand() = withName("Stop motor") {
	runOnce { stopMotor() }
}

// --- Angle commands ---

fun IntakeSubsystem.retractIntakeCommand() = withName("Retract intake") {
	runOnce { retractIntake() }
}

fun IntakeSubsystem.deployIntakeCommand() = withName("Deploy intake") {
	runOnce { deployIntake() }
}

enum class IntakeState(val exitCondition: () -> Boolean) {
	Deploying({
		IntakeSubsystem.isWithinAngleTolerance
	}),
	Intaking({
		IntakeSubsystem.isMotorCurrentAboveThreshold
	}),
	Retracting({
		false //GrabberSubsystem.hasNote
	}),
	Finished({
		false
	}),
}

fun IntakeSubsystem.intakeCommand() = withName("Intake") {
	var intakeState = IntakeState.Deploying
	run {
		if (intakeState == Deploying) {
			deployIntake()
			if (intakeState.exitCondition()) intakeState = Intaking
		}
		if (intakeState == Intaking) {
			runMotor()
			if (intakeState.exitCondition()) intakeState = Retracting
		}
		if (intakeState == Retracting) {
			retractIntake()
			if (intakeState.exitCondition()) intakeState = Finished
		}
		if (intakeState == Finished) {
			stopMotor()
		}
	} until { intakeState == Finished }
}
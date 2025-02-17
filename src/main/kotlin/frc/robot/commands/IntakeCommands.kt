package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem

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

fun IntakeSubsystem.maintainAngleCommand(angle: () -> Rotation2d) = withName("Maintain angle") {
	run {
		setAngle(angle())
		stopWheelMotor()
	}
}

// --- Intaking commands ---

/** Intakes from ground, does not end automatically. */
fun IntakeSubsystem.intakeCommand() = withName("Intake from ground") {
	run {
		setAngle(IntakeConstants.DEPLOYED_ANGLE)
		stopWheelMotor()
	} until { angleError.absoluteValue <= Rotation2d.fromDegrees(25.0) } andThen run {
		stopAngleMotor()
		runWheelMotor()
	} until { isBeamBreakInterfered } finallyDo { stopWheelMotor() }
}

fun IntakeSubsystem.feedToGrabberCommand() = withName("Feed to grabber") {
	run {
		setAngle(IntakeConstants.RETRACTED_ANGLE)
		if (isWithinAngleTolerance) runWheelMotor()
	} finallyDo { stopWheelMotor() }
}

fun IntakeSubsystem.ejectToL1Command() = withName("Eject to L1") {
	IntakeSubsystem.run {
		setAngle(IntakeConstants.L1_ANGLE)
		stopWheelMotor()
	} until { angleError.absoluteValue <= Rotation2d.fromDegrees(25.0) } andThen (
		IntakeSubsystem.run {
			stopAngleMotor()
			runWheelMotorReverse()
		} withTimeout(2.0)
	)
}

// --- Testing commands ---

/** Use for testing. */
fun IntakeSubsystem.test_openLoopRunWheelsCommand(voltage: () -> Volts) = withName("Open loop run wheels") {
	run { setWheelMotorVoltage(voltage()) }
}

/** Use for testing. */
fun IntakeSubsystem.test_openLoopRunAngleControlCommand(voltage: () -> Volts) = withName("Open loop angle control") {
	run { setAngleMotorVoltage(voltage()) }
}

package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem

// --- Wheels commands ---

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
	} until { angleError.absoluteValue <= Rotation2d.fromDegrees(25.0) } andThen (run {
		stopAngleMotor()
		setWheelMotorVoltage(IntakeConstants.INTAKING_VOLTAGE)
	} until { isAtMaxAngle && isBeamBreakInterfered }) finallyDo { stopWheelMotor() }
}

fun IntakeSubsystem.feedToGrabberCommand() = withName("Feed to grabber") {
	run {
		setAngle(IntakeConstants.FEEDING_ANGLE)
		if (isWithinAngleTolerance) setWheelMotorVoltage(IntakeConstants.INTAKING_VOLTAGE) else stopWheelMotor()
	} finallyDo { stopWheelMotor() }
}

fun IntakeSubsystem.ejectToL1Command() = withName("Eject to L1") {
	IntakeSubsystem.run {
		setAngle(IntakeConstants.L1_ANGLE)
		stopWheelMotor()
	} until { angleError.absoluteValue <= Rotation2d.fromDegrees(25.0) } andThen (
		IntakeSubsystem.run {
			stopAngleMotor()
			setWheelMotorVoltage(IntakeConstants.EJECTING_VOLTAGE)
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

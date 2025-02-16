package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.IntakeState.*
import frc.robot.commands.IntakeState.DEPLOYING
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

fun IntakeSubsystem.maintainL1AngleCommand(): Command = withName("Maintain L1 angle") {
	run { maintainL1AngleCommand() }
}

/** Maintains a retracted angle. Does not end automatically. */
fun IntakeSubsystem.maintainIntakeRetractedCommand(): Command = withName("Retract intake") {
	run { maintainIntakeRetractedCommand() }
}

/** Sets the intake angle to the deployed angle. Ends instantly. */
fun IntakeSubsystem.maintainIntakeDeployedCommand(): Command = withName("Deploy intake") {
	run { maintainIntakeDeployedCommand() }
}

// --- Intake command ---

private enum class IntakeState(val shouldExitState: () -> Boolean) {
	DEPLOYING(shouldExitState = {
		IntakeSubsystem.angleError.absoluteValue <= Rotation2d.fromDegrees(25.0)
	}),
	INTAKING(shouldExitState = {
		IntakeSubsystem.isBeamBreakInterfered
	}),
	RETRACTING(shouldExitState = {
		IntakeSubsystem.isWithinAngleTolerance
	}),
	FEEDING(shouldExitState = {
		false
	}),
}

/** Intakes from ground, does not end automatically. */
fun IntakeSubsystem.intakeCommand() = withName("Intake") {
	var intakeState = DEPLOYING
	runOnce { intakeState = DEPLOYING } andThen
	run {
		when (intakeState) {
			DEPLOYING -> {
				setAngleToDeploy()
				stopWheelMotor()
				if (intakeState.shouldExitState()) intakeState = INTAKING
			}
			INTAKING -> {
				SmartDashboard.putString("Reached Intaking", "wawa")
				stopAngleMotor()
				runWheelMotor()
				if (intakeState.shouldExitState()) {
					intakeState = RETRACTING
					stopWheelMotor()
				}
			}
			RETRACTING -> {
				stopWheelMotor()
				setAngleToRetracted()
				if (intakeState.shouldExitState()) {
					intakeState = FEEDING
				}
			}
			FEEDING -> {
				setAngleToRetracted()
				runWheelMotor()
			}
		}
	} finallyDo { stopWheelMotor() }
}

fun IntakeSubsystem.ejectToL1Command() = withName("Eject to L1") {
	IntakeSubsystem.run {
		setAngleToL1()
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

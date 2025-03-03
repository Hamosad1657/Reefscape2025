package frc.robot.commands

import com.hamosad1657.lib.commands.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.EXTENDING_VOLTAGE
import frc.robot.subsystems.climbing.ClimbingConstants.RETRACTING_VOLTAGE
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import kotlin.time.DurationUnit.SECONDS

fun ClimbingSubsystem.extendClimbingSystemCommand(): Command = withName("Extend") {
	run { setAngleMotorVoltage(EXTENDING_VOLTAGE) }
}

fun ClimbingSubsystem.retractClimbingSystemCommand(): Command = withName("Retract") {
	run { setAngleMotorVoltage(RETRACTING_VOLTAGE) }
}

fun extendClimbingAndIntakeSystemsCommand(): Command = withName("Extend climbing and intake systems") {
	IntakeSubsystem.maintainAngleCommand { IntakeConstants.PARALLEL_TO_FLOOR_ANGLE }.withTimeout(
		IntakeConstants.EXTENDING_INTAKE_TO_CLIMB_TIMEOUT.toDouble(SECONDS)) andThen {
		ClimbingSubsystem.extendClimbingSystemCommand()
	}
}


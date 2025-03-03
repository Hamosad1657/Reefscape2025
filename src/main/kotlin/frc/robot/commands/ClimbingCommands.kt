package frc.robot.commands

import com.hamosad1657.lib.commands.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.EXTENDING_VOLTAGE
import frc.robot.subsystems.climbing.ClimbingConstants.RETRACTING_VOLTAGE
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

fun ClimbingSubsystem.extendClimbingSystem(): Command = withName("extend ") {
	run { setAngleMotorVoltage(EXTENDING_VOLTAGE) }
}

fun ClimbingSubsystem.retractClimbingSystem(): Command = withName("extend ") {
	run { setAngleMotorVoltage(RETRACTING_VOLTAGE) }
}


package frc.robot.commands

import com.hamosad1657.lib.commands.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

fun ClimbingSubsystem.grabCage(): Command = withName("climb") {
	run { setWheelsMotorVoltage(Constants.GRABBING_CAGE_VOLTAGE) } withTimeout (1.0)
}

fun ClimbingSubsystem.setClimberAngle(setpoint: Rotation2d): Command = withName("climb") {
	run { setAngleMotorSetpoint(setpoint) } until { isWithinAngleTolerance }
}

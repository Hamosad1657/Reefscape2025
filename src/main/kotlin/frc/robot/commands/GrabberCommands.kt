package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

fun GrabberSubsystem.runInwardsCommand(): Command = withName("run inwards") {
	run { setWheelsVoltage(GrabberConstants.WHEELS_INTAKE_VOLTAGE) } finallyDo {
		stopWheelsMotor()
	}
}

fun GrabberSubsystem.runOutwardsCommand(): Command = withName("run outwards") {
	run { setWheelsVoltage(GrabberConstants.WHEELS_EJECT_VOLTAGE) } finallyDo {
		stopWheelsMotor()
		didCoralEnterBeamBreak = false
	}
}

fun GrabberSubsystem.getToAngleCommand(angle: Rotation2d): Command = withName("get to angle") {
	runOnce{ getToAngleWithLimits(angle) }
}

fun GrabberSubsystem.getToAngleAndEndCommand(angle: Rotation2d): Command = withName("get to angle and end") {
	getToAngleCommand(angle) until {
		isAngleWithinTolerance
	}
}

// I feel like there are still more commands to add. For example, I need a get to angle command that ends when the grabber is in angle tolerance.

//--- Test commands ---

fun GrabberSubsystem.setWheelsVoltageCommand(voltage: Volts): Command = withName("set wheels voltage") {
	run { setWheelsVoltage(voltage) }
}

fun GrabberSubsystem.setAngleMotorVoltageCommand(voltage: Volts): Command = withName("set angle motor voltage") {
	run { setAngleVoltage(voltage) }
}
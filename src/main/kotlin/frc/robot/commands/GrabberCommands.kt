package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem

fun GrabberSubsystem.grabCommand(): Command = withName("grab") { // Maybe a better name would be run inwards
	run { setWheelsVoltage(GrabberConstants.WHEELS_GRAB_VOLTAGE) } finallyDo {
		stopWheelsMotor()
	}
}

// You have a command that never ends in a sequential command group. It will never get to the next thing in the sequence.
fun GrabberSubsystem.ejectCommand(): Command = withName("eject") { // And for here run outwards
	run { setWheelsVoltage(GrabberConstants.WHEELS_EJECT_VOLTAGE) } andThen {
		runOnce { isCoralDetected = false }
	} finallyDo {
		stopWheelsMotor()
	}
}

// I think it would be preferable to just have a command that instantly sets a new angle setpoint and ends.
fun GrabberSubsystem.getToAngleCommand(angle: Rotation2d): Command = withName("get to angle")  {
	run { getToAngleWithLimits(angle) } finallyDo {
		stopAngleMotor()
	}
}

fun GrabberSubsystem.intakeCommand(): Command = withName("intake")	{
		// This would not work, you can't run 2 commands from the same subsystem simultaneously and it's bad design.
		// The commands need to not be in groups, unless a sequential group which is fine.
	getToAngleCommand(GrabberConstants.GRABBING_ANGLE) until { isAngleWithinTolerance } alongWith grabCommand() until {
		isCoralDetected
	}
}

// This command has no use. Remove it.
fun GrabberSubsystem.getReadyToPlaceCommand(angle: Rotation2d): Command = withName("get ready to place") {
	SequentialCommandGroup(
		intakeCommand(),
		getToAngleCommand(angle) until { isAngleWithinTolerance }
	) finallyDo {
		stopAngleMotor()
		stopWheelsMotor()
	}
}

// How is this different than ejectCommand? why does it take an angle and not use it?
fun GrabberSubsystem.placeCoralCommand(angle: Rotation2d): Command = withName("place coral") {
	SequentialCommandGroup(
		ejectCommand() until { !isCoralDetected }, // The commands provide the logic for this, it will never become false.
	) finallyDo {
		stopAngleMotor()
		stopWheelsMotor()
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
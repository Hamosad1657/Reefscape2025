package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.grabber.GrabberConstants as Constants
import frc.robot.subsystems.grabber.GrabberSubsystem

object GrabberCommands {
	fun GrabberSubsystem.grabCommand(): Command = withName("grab") {
		run { setWheelsVoltage(Constants.WHEELS_GRAB_VOLTAGE) } finallyDo {
			stopWheelsMotor()
		}
	}

	fun GrabberSubsystem.ejectCommand(): Command = withName("eject") {
		run { setWheelsVoltage(Constants.WHEELS_EJECT_VOLTAGE) } finallyDo {
			stopWheelsMotor()
		}
	}

	fun GrabberSubsystem.getToAngleCommand(angle: Rotation2d): Command = withName("get to angle")  {
		run { getToAngleWithLimits(angle) } finallyDo {
			stopAngleMotor()
		}
	}

	fun GrabberSubsystem.intakeCommand(): Command = withName("intake")	{
		getToAngleCommand(Constants.GRABBING_ANGLE) until { isAngleWithinTolerance } alongWith grabCommand() until {
			isCoralDetected
		}
	}

	fun GrabberSubsystem.getReadyToPlaceCommand(angle: Rotation2d): Command = withName("get ready to place command") {
		SequentialCommandGroup(
			intakeCommand(),
			getToAngleCommand(angle) until { isAngleWithinTolerance }
		) finallyDo {
			stopAngleMotor()
			stopWheelsMotor()
		}
	}

	fun GrabberSubsystem.placeCoralCommand(angle: Rotation2d): Command = withName("place coral") {
		SequentialCommandGroup(
			ejectCommand() until { !isCoralDetected },
			runOnce{ isCoralDetected = false },
		) finallyDo {
			stopAngleMotor()
			stopWheelsMotor()
		}
	}

	//--- Tests commands ---

	fun GrabberSubsystem.setWheelsVoltageCommand(voltage: Volts): Command = withName("set wheels voltage") {
		run { setWheelsVoltage(voltage) }
	}

	fun GrabberSubsystem.setAngleMotorVoltageCommand(voltage: Volts): Command = withName("set angle motor voltage") {
		run { setAngleVoltage(voltage) }
	}
}

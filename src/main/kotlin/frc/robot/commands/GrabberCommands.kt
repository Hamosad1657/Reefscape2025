package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.grabber.GrabberConstants as Constants
import frc.robot.subsystems.grabber.GrabberSubsystem

object GrabberCommands {
	fun GrabberSubsystem.setWheelsSpeedCommand(output: PercentOutput): Command = withName("set wheels speed") {
		run { setWheelsSpeed(output) }
	}

	fun GrabberSubsystem.setAngleMotorSpeedCommand(output: PercentOutput): Command = withName("set angle motor speed") {
		run { setAngleSpeed(output) }
	}

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

	fun GrabberSubsystem.intakeCommand(angle: Rotation2d): Command = withName("intake")	{
	SequentialCommandGroup(
		getToAngleCommand(Constants.GRABBING_ANGLE) until { isAngleWithinTolerance } alongWith grabCommand(),
		wait(Constants.GRAB_DURATION_SEC),
		)
	}

	fun GrabberSubsystem.placeCoralCommand(angle: Rotation2d): Command = withName("place coral") {
		SequentialCommandGroup(
			intakeCommand(angle),
			getToAngleCommand(angle) until { isAngleWithinTolerance },
			ejectCommand(),
		) finallyDo {
			stopAngleMotor()
			stopWheelsMotor()
		}
	}
}
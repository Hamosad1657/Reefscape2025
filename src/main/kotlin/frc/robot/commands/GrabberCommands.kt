package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.grabber.GrabberConstants as Constants
import frc.robot.subsystems.grabber.GrabberSubsystem

object GrabberCommands {
	fun GrabberSubsystem.setWheelsSpeedCommand(output: PercentOutput): Command {
		return run { setWheelsSpeed(output) }
	}

	fun GrabberSubsystem.setAngleMotorSpeedCommand(output: PercentOutput): Command {
		return run { setAngleSpeed(output) }
	}

	fun GrabberSubsystem.grabCommand(): Command {
		return run { setWheelsVoltage(Constants.WHEELS_GRAB_VOLTAGE) } finallyDo {
			stopWheelsMotor()
		}
	}

	fun GrabberSubsystem.ejectCommand(): Command {
		return run { setWheelsVoltage(Constants.WHEELS_EJECT_VOLTAGE) } finallyDo {
			stopWheelsMotor()
		}
	}

	fun GrabberSubsystem.getToAngleCommand(angle: Rotation2d): Command {
		return run { getToAngleWithLimits(angle) } finallyDo {
			stopAngleMotor()
		}
	}
}
package frc.robot.commands

import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.elevator.ElevatorSubsystem

fun ElevatorSubsystem.setHeightCommand(height: Length): Command {
	return run { setHeight(height) }
}

fun ElevatorSubsystem.setElevatorMotorsCommand(voltage: Volts): Command {
	return run { setElevatorMotorsVolts(voltage) }
}
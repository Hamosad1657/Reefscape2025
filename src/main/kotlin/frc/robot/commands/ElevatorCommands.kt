package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.elevator.ElevatorSubsystem

fun ElevatorSubsystem.setHeightCommand(height: Length): Command = withName("set height") {
	run { setHeight(height) }
}

fun ElevatorSubsystem.setElevatorMotorsVoltageCommand(voltage: Volts): Command = withName("set elevator motors voltage") {
	run { setElevatorMotorsVolts(voltage) }
}
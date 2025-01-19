package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.elevator.ElevatorSubsystem

/** Use only for testing */
fun ElevatorSubsystem.setElevatorMotorsVoltageCommand(voltage: Volts): Command = withName("Set elevator motors voltage") {
	run { setElevatorMotorsVoltage(voltage) }
}

fun ElevatorSubsystem.setHeightCommand(height: Length): Command = withName("Set height") {
	run { setHeight(height) }
}

fun ElevatorSubsystem.setHeightUntilReachedCommand(height: Length): Command = withName("Set height until reached") {
	setHeightCommand(height) until { isAtHeight }
}
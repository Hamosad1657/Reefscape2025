package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.elevator.ElevatorSubsystem

/** Sets elevator height setpoint. Ends instantly. */
fun ElevatorSubsystem.setHeightCommand(height: Length): Command = withName("Set height") {
	runOnce { setHeight(height) }
}

// --- Testing ---

fun ElevatorSubsystem.test_elevatorSetVoltageCommand(voltage: Volts): Command = withName("Set elevator motors voltage") {
	run { setElevatorMotorsVoltage(voltage) }
}

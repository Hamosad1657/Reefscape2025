package frc.robot.commands

import com.hamosad1657.lib.commands.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingSubsystem

fun ClimbingSubsystem.climbCommand(): Command = withName("climb") {
	run { climb() }
}
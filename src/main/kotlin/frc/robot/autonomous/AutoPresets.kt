package frc.robot.autonomous

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import frc.robot.commands.*
import frc.robot.commands.GrabberEjectMode.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

fun scoreL1GHTopRoutine() = withName("L1 GH top routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) alongWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("GHL1ScoreRight")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}
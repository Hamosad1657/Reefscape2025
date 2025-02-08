package frc.robot.autonomous

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import frc.robot.commands.*
import frc.robot.commands.GrabberEjectMode.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

fun scoreL1GHLeftTopRoutine() = withName("L1 GH left top routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("GHL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1GHRightTopRoutine() = withName("L1 GH right top routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("GHL1ScoreRight")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL4GHLeftTopRoutine() = withName("L4 GH left top routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L4, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("GHL4ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L4) withTimeout(3.0)))
}

fun scoreL4GHRightTopRoutine() = withName("L4 GH right top routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L4, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("GHL4ScoreRight")) andThen
			(GrabberSubsystem.ejectCommand(L4) withTimeout(3.0)))
}

fun scoreL1IJLeftTopRoutinePosition6() = withName("L1 IJ left Position6 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position6IJL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1IJRightTopRoutinePosition6() = withName("L1 IJ right Position6 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position6IJL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1IJLeftTopRoutinePosition5() = withName("L1 IJ left Position5 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position5IJL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1IJRightTopRoutinePosition5() = withName("L1 EF right Position5 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position5IJL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1EFLeftTopRoutinePosition2() = withName("L1 EF left Position2 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1EFRightTopRoutinePosition2() = withName("L1 EF right Position2 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1EFLeftTopRoutinePosition1() = withName("L1 EF left Position1 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}

fun scoreL1EFRightTopRoutinePosition1() = withName("L1 EF right Position1 routine") {
	ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.L1, true) raceWith
		( SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft")) andThen
			(GrabberSubsystem.ejectCommand(L1) withTimeout(3.0)))
}
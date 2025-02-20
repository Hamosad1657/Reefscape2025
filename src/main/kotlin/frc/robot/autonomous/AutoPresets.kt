package frc.robot.autonomous

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.Robot
import frc.robot.commands.*
import frc.robot.commands.GrabberVoltageMode.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

fun goTwoMetersForwardRoutine(position: Int) = withName("go two meters forward") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile(("position${position}GoForward")), Robot.alliance == Alliance.Red)
}

fun scoreL1GHLeftTopRoutine() = withName("L1 GH left top routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GHL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1GHRightTopRoutine() = withName("L1 GH right top routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GHL1ScoreRight"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL4GHLeftTopRoutine() = withName("L4 GH left top routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L4) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GHL4ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L4) withTimeout(3.0)))
}

fun scoreL4GHRightTopRoutine() = withName("L4 GH right top routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L4) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GHL4ScoreRight"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L4) withTimeout(3.0)))
}

fun scoreL1IJLeftTopRoutinePosition6() = withName("L1 IJ left Position6 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position6IJL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1IJRightTopRoutinePosition6() = withName("L1 IJ right Position6 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position6IJL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1IJLeftTopRoutinePosition5() = withName("L1 IJ left Position5 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position5IJL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1IJRightTopRoutinePosition5() = withName("L1 EF right Position5 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position5IJL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1EFLeftTopRoutinePosition2() = withName("L1 EF left Position2 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1EFRightTopRoutinePosition2() = withName("L1 EF right Position2 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1EFLeftTopRoutinePosition1() = withName("L1 EF left Position1 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1EFRightTopRoutinePosition1() = withName("L1 EF right Position1 routine") {
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position1EFL1ScoreLeft"), Robot.alliance == Alliance.Red) andThen
			(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}
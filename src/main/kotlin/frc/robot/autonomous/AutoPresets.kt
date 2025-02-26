package frc.robot.autonomous

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.Robot
import frc.robot.commands.*
import frc.robot.commands.GrabberVoltageMode.*
import frc.robot.field.Pipe
import frc.robot.field.ReefSide
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

fun goTwoMetersForwardRoutine(position: Int) = withName("go two meters forward") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile(("position${position}GoForward")), Robot.alliance == Alliance.Red)
}

fun scoreL1Routine(pipe: Pipe, position: Int) = withName("$position score L1 ${pipe.letter}") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position${position}${pipe.letter}L1Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
	(GrabberSubsystem.setVoltageCommand(true, false, EJECT_TO_L1) withTimeout(3.0)))
}

fun scoreL1Intake(reefSide: ReefSide, position: Int) = withName("Score L1 $reefSide from $position") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position${position}${reefSide}L1Intake"), Robot.alliance == Alliance.Red) andThen
	IntakeSubsystem.ejectToL1Command(true)
}

fun HL1ScoreRoutine() = withName("L1 GH right routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("HL1Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		(GrabberSubsystem.setVoltageCommand(true, false, EJECT_TO_L1) withTimeout(3.0)))
}

fun GL1ScoreRoutine() = withName("L1 GH left routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GL1Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) raceWith
		(GrabberSubsystem.setVoltageCommand(true, false, EJECT_TO_L1) withTimeout(3.0)))
}

fun GL4ScoreRoutine() = withName("L4 GH left top routine") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GL4Score"), Robot.alliance == Alliance.Red) alongWith
		(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L4) raceWith
		(waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen  (GrabberSubsystem.setVoltageCommand(true, false, EJECT_TO_L4) withTimeout(3.0))))
}

fun HL4ScoreRoutine() = withName("L4 GH right top routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("HL4Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L4) raceWith
		(GrabberSubsystem.setVoltageCommand(true, false, EJECT_TO_L1) withTimeout(3.0)))
}

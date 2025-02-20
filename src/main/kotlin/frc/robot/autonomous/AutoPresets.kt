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
import java.nio.channels.Pipe

fun goTwoMetersForwardRoutine(position: Int) = withName("go two meters forward") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile(("position${position}GoForward")), Robot.alliance == Alliance.Red)
}

fun scoreL1Routine(pipe: frc.robot.field.Pipe, position: Int) = withName("$position score L1 ${pipe.letter}") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position${position}${pipe.letter}L1Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) andThen
	(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun HL1ScoreRoutine() = withName("L1 GH right routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("HL1Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) andThen
	(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun GL1ScoreRoutine() = withName("L1 GH left routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GL1Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) andThen
	(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun GL4ScoreRoutine() = withName("L4 GH left top routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("GL4Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) andThen
	(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

fun HL4ScoreRoutine() = withName("L4 GH right top routine") {
	( SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("HL4Score"), Robot.alliance == Alliance.Red) andThen
	JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.L1) andThen
	(GrabberSubsystem.setVoltageCommand(true, EJECT_TO_L1) withTimeout(3.0)))
}

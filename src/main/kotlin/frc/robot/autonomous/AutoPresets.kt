package frc.robot.autonomous

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.degrees
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.Robot
import frc.robot.commands.*
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

fun goForwardRoutine() = withName("Go forward") {
	SwerveSubsystem.runOnce {
		if (Robot.alliance == Red) SwerveSubsystem.zeroGyro() else SwerveSubsystem.setGyro(180.0.degrees)
	} andThen SwerveSubsystem.run { SwerveSubsystem.setChassisSpeeds(
		ChassisSpeeds(1.0, 0.0, 0.0)
	) } withTimeout(0.8)
}

fun scoreL1IntakePosition1Routine() = withName("Score L1 EF from position 1") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position1EFL1Intake"), Robot.alliance == Alliance.Red) andThen
		IntakeSubsystem.ejectToL1Command(true)
}
fun scoreL1IntakePosition2Routine() = withName("Score L1 EF from position 2") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position2EFL1Intake"), Robot.alliance == Alliance.Red) andThen
		IntakeSubsystem.ejectToL1Command(true)
}
fun scoreL1IntakePosition3Routine() = withName("Score L1 GH from position 3") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position3GHL1Intake"), Robot.alliance == Alliance.Red) andThen
		IntakeSubsystem.ejectToL1Command(true)
}
fun scoreL1IntakePosition4Routine() = withName("Score L1 GH from position 4") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position4GHL1Intake"), Robot.alliance == Alliance.Red) andThen
		IntakeSubsystem.ejectToL1Command(true)
}
fun scoreL1IntakePosition5Routine() = withName("Score L1 IJ from position 5") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position5IJL1Intake"), Robot.alliance == Alliance.Red) andThen
		IntakeSubsystem.ejectToL1Command(true)
}
fun scoreL1IntakePosition6Routine() = withName("Score L1 IJ from position 6") {
	SwerveSubsystem.followInitialPathCommand(PathPlannerPath.fromPathFile("Position6IJL1Intake"), Robot.alliance == Alliance.Red) andThen
		IntakeSubsystem.ejectToL1Command(true)
}
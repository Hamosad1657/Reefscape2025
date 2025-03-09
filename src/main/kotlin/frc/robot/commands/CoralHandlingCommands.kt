package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Mps
import com.hamosad1657.lib.units.Seconds
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.vision.CoralVision

fun loadCoralFromIntake() = withName("Load coral from intake") {
	((IntakeSubsystem.maintainAngleCommand { IntakeConstants.FEEDING_ANGLE } until { IntakeSubsystem.isWithinAngleTolerance }) andThen
	  (JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false) { JointedElevatorState.INTAKE } raceWith (
	waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen (
		IntakeSubsystem.feedToGrabberCommand() raceWith GrabberSubsystem.loadFromIntakeCommand() finallyDo {
			LEDsSubsystem.currentMode = ACTION_FINISHED
		}
		)))).withInterruptBehavior(kCancelIncoming)
}

fun intakeCoralFromGroundCommand() = withName("Intake coral from ground") {
	(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false) { JointedElevatorState.RESTING } raceWith
		(
			(if (!IntakeSubsystem.isBeamBreakInterfered) IntakeSubsystem.intakeCommand(true) else instantCommand{}) andThen
				(IntakeSubsystem.maintainAngleCommand { IntakeConstants.FEEDING_ANGLE } until { IntakeSubsystem.isWithinAngleTolerance })
			)) andThen (JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false) { JointedElevatorState.INTAKE } raceWith (
				waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen (
					IntakeSubsystem.feedToGrabberCommand() raceWith GrabberSubsystem.loadFromIntakeCommand() finallyDo {
						LEDsSubsystem.currentMode = ACTION_FINISHED
					}
				))).withInterruptBehavior(kCancelIncoming)
}

private val autoIntakeFromGroundScanAngularVelocity = AngularVelocity.fromRps(0.1)
private const val autoIntakeFromGroundScanPeriod: Seconds = 1.5
private const val autoIntakeFromGroundVelocity: Mps = 1.0

fun autoIntakeCoralFromGroundCommand() = withName("Auto intake coral from ground") {
	val timer = Timer()
	timer.reset()
	timer.stop()
	var swerveDriveTime: Seconds = 0.0
	(SwerveSubsystem.oscillateCommand(
		autoIntakeFromGroundScanAngularVelocity,
		autoIntakeFromGroundScanPeriod,
	) until { CoralVision.hasTargets }) andThen
		((SwerveSubsystem.rotateToCoralUntilLockedCommand()) andThen
			((
				IntakeSubsystem.intakeCommand(true)
				) alongWith (wait(0.4) andThen Commands.runOnce(
				{ swerveDriveTime = CoralVision.calculateXToCoral().asMeters / autoIntakeFromGroundVelocity }) andThen (SwerveSubsystem.run {
					timer.start()
					SwerveSubsystem.setChassisSpeeds(
						ChassisSpeeds(0.0, autoIntakeFromGroundVelocity, 0.0)
					)
			} until { timer.hasElapsed((swerveDriveTime / 1.3)) } andThen
				SwerveSubsystem.runOnce { SwerveSubsystem.setChassisSpeeds(ChassisSpeeds()) })
			)))
}

fun intakeCoralFromCoralStationCommand() = withName("Load coral from coral station") {
	(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true, JointedElevatorState.CORAL_STATION) raceWith
		(waitUntil { JointedElevatorSubsystem.isWithinTolerance } andThen
			(GrabberSubsystem.loadFromCoralStationCommand() alongWith LEDsSubsystem.runOnce {
			LEDsSubsystem.currentMode = INTAKING
		})) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED }).withInterruptBehavior(kCancelIncoming)
}
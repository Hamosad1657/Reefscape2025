package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import frc.robot.autonomous.generatePathAroundReef
import frc.robot.commands.*
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * A segment which starts at a side of the reef, Goes to one of the coral stations and returns to KL/CD.
 *
 * @param startingSide - The side to start from.
 * @param goToKL - Which coral station to go to. True means the KL coral station, and false means the CD coral station.
 */
class CoralStationSegment(
	startingSide: ReefSide,
	val goToKL: Boolean, // TODO
	val isClockwise: Boolean,
): AutonomousSegment(startingSide) {
	override fun generateCommand() = withName("Get to coral station ${if (goToKL) "KL" else "CD"}") {
		SwerveSubsystem.followPathCommand(generatePathAroundReef(
			startingSide,
			if (goToKL) ReefSide.KL else ReefSide.CD,
			shouldStartClose = false,
			shouldEndClose = false,
			far = true,
			useQuickest = false,
			clockwise = isClockwise,
		)) andThen SwerveSubsystem.followPathCommand(
			if (goToKL) PathPlannerPath.fromPathFile("KL to coral station") else PathPlannerPath.fromPathFile("CD to coral station")
		) andThen (
			SwerveSubsystem.alignToPoseCommand({Pose2d()}, true) alongWith // TODO: Make align to coral station command
			intakeCoralFromCoralStationCommand()
		) andThen SwerveSubsystem.followPathCommand(
			if (goToKL) PathPlannerPath.fromPathFile("Coral station to KL") else PathPlannerPath.fromPathFile("Coral station to CD")
		)
	}
}
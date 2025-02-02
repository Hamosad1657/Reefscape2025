package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.commands.*
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * A segment which starts at a side of the reef, Goes to one of the coral stations and returns to One of the 3 sides close to it.
 *
 * @param goToKL - Which coral station to go to. True means the KL coral station, and false means the CD coral station.
 * @param startingSide - The side from which a path to the coral station will be followed. When the coral station is KL, this can only be
 * AB, KL or IJ. When the coral station is CD this can only be AB, CD or EF.
 * @param endingSide - The side to which the robot will return to from the coral station. When the coral station is KL, this can only be
 * AB, KL or IJ. When the coral station is CD this can only be AB, CD or EF.
 * @param isClockwise - Whether the routine gets to [startingSide] by going clockwise around the reef or counterclockwise.
 */
class CoralStationSegment(
	private val goToKL: Boolean,
	startingSide: ReefSide,
	endingSide: ReefSide,
	isClockwise: Boolean,
): AutonomousSegment(startingSide, endingSide, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName("Get to coral station ${if (goToKL) "KL" else "CD"}") {
		SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("${startingSide.sideName}-far to ${if (goToKL) "KL" else "CD"} coral station")
		) andThen (
			SwerveSubsystem.alignToPoseCommand({Pose2d()}, true) alongWith // TODO: Make align to coral station command
			intakeCoralFromCoralStationCommand() // TODO: Add align back to The far side of the coral station
		) andThen SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("${if (goToKL) "KL" else "CD"} coral station to ${startingSide.sideName}-far")
		)
	}
}
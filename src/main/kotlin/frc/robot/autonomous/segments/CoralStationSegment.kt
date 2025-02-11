package frc.robot.autonomous.segments

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.commands.*
import frc.robot.field.CoralStation
import frc.robot.field.ReefSide
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * A segment which starts at a side of the reef, Goes to one of the coral stations and returns to One of the 3 sides close to it.
 *
 * @param coralStation - Which coral station to go to.
 * @param startingSide - The side from which a path to the coral station will be followed. When the coral station is KL, this can only be
 * AB, KL or IJ. When the coral station is CD this can only be AB, CD or EF.
 * @param endingSide - The side to which the robot will return to from the coral station. When the coral station is KL, this can only be
 * AB, KL or IJ. When the coral station is CD this can only be AB, CD or EF.
 * @param isClockwise - Whether the routine gets to [startingSide] by going clockwise around the reef or counterclockwise.
 */
class CoralStationSegment(
	private val coralStation: CoralStation,
	startingSide: ReefSide,
	endingSide: ReefSide,
	isClockwise: Boolean,
): AutonomousSegment(startingSide, endingSide, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName("Get to coral station ${coralStation.name}") {
		SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("${startingSide.name}-far to ${if (coralStation == CoralStation.KL) "KL" else "CD"} coral station"),
			alliance == Alliance.Red,
		) andThen (
			SwerveSubsystem.alignToCoralStation(coralStation, alliance) alongWith
			intakeCoralFromCoralStationCommand()
		) andThen SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("${coralStation.name} coral station to ${startingSide.name}-far"),
			alliance == Alliance.Red,
		)
	}
}
package frc.robot.autonomous

import com.hamosad1657.lib.units.degrees
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.IdealStartingState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PointTowardsZone
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kError
import frc.robot.field.FieldConstants
import frc.robot.field.ReefSide
import frc.robot.subsystems.swerve.SwerveConstants

/**
 * Returns true if it is quicker to get to [endSide] from [startSide] by going clockwise.
 */
fun findQuickestDirection(startSide: ReefSide, endSide: ReefSide): Boolean {
	val delta = if (endSide.number >= startSide.number) endSide.number - startSide.number else startSide.number - endSide.number
	 return if (endSide.number >= startSide.number) delta >= 3 else delta <= 3
}

/**
 * Creates a list of poses for creating a path between two sides of the reef.
 *
 * @param startSide - The side of the reef the list should start from.
 * @param endSide - The side of the reef the list should end on.
 * @param far - Whether the poses should be far from the reef or close to it.
 * @param useQuickest - Whether to choose if to go clockwise or counterclockwise automatically or not. If set to false a
 * value for [clockwise] must be given.
 * @param clockwise - If not choosing path automatically, this decides in what direction around the reef the path would go.
 */
fun findPosesBetweenReefSides(startSide: ReefSide, endSide: ReefSide, far: Boolean, useQuickest: Boolean, clockwise: Boolean = false): List<Pose2d> {
	if (startSide.number !in 0..5 || endSide.number !in 0..5) {
		Alert("Invalid path generation request", kError)
		return listOf()
	}

	val delta = if (endSide.number >= startSide.number) endSide.number - startSide.number else startSide.number - endSide.number
	val isClockwiseQuickest = findQuickestDirection(startSide, endSide)

	val numberOfSteps = if (delta > 3) 6-delta else delta

	val poses = MutableList(0) { Pose2d() }
	if ((isClockwiseQuickest && useQuickest) || (!useQuickest && clockwise) ) {
		for (i in startSide.number downTo(startSide.number-numberOfSteps)) {
			val poseIndex = if (i < 0) (6 + i) * 2 else i * 2
			val nextPoseIndex = if (poseIndex == 0) 11 else poseIndex - 1
			if (far) {
				poses.add(FieldConstants.Poses.FAR_POSES[(poseIndex)].let { Pose2d(it.x, it.y, it.rotation + 90.0.degrees) })
				if (i != startSide.number-numberOfSteps) poses.add(FieldConstants.Poses.FAR_POSES[nextPoseIndex].let { Pose2d(it.x, it.y, it.rotation + 90.0.degrees) })
			} else {
				poses.add(FieldConstants.Poses.CLOSE_POSES[poseIndex].let { Pose2d(it.x, it.y, it.rotation + 90.0.degrees) })
				if (i != startSide.number-numberOfSteps) poses.add(FieldConstants.Poses.CLOSE_POSES[nextPoseIndex].let { Pose2d(it.x, it.y, it.rotation + 90.0.degrees) })
			}
		}
	} else {
		for (i in startSide.number..startSide.number+numberOfSteps) {
			val poseIndex = if (i > 5) (i - 6) * 2 else i * 2
			if (far) {
				poses.add(FieldConstants.Poses.FAR_POSES[poseIndex].let { Pose2d(it.x, it.y, it.rotation - 90.0.degrees) })
				if (i != startSide.number+numberOfSteps) poses.add(FieldConstants.Poses.FAR_POSES[poseIndex + 1].let { Pose2d(it.x, it.y, it.rotation - 90.0.degrees) })
			} else {
				poses.add(FieldConstants.Poses.CLOSE_POSES[poseIndex].let { Pose2d(it.x, it.y, it.rotation - 90.0.degrees) })
				if (i != startSide.number+numberOfSteps) poses.add(FieldConstants.Poses.CLOSE_POSES[poseIndex + 1].let { Pose2d(it.x, it.y, it.rotation - 90.0.degrees) })
			}
		}
	}
	return poses
}

/**
 * Creates a path around the reef
 *
 * @param startSide - The side of the reef the list should start from. 0 is AB and the numbers go counter clockwise.
 * (1 is CD, 2 is EF, and so on until 5 which is KL).
 * @param endSide - The side of the reef the list should end on, in the same format.
 * @param startClose - Whether the path should start close to the reef or not.
 * @param endClose - whether the path should end close to the reef or not.
 * @param far - Whether the path should go around close to the reef or far from it.
 * @param useQuickest - Whether to choose if to go clockwise or counterclockwise automatically or not. If set to false a
 * value for [isClockwise] must be given.
 * @param isClockwise - If not choosing path automatically, this decides in what direction around the reef the path would go.
 */
fun generatePathAroundReef(
	startSide: ReefSide,
	endSide: ReefSide,
	startClose: Boolean,
	endClose: Boolean,
	far: Boolean,
	useQuickest: Boolean,
	isClockwise: Boolean = false,
): PathPlannerPath {
	val delta = if (endSide.number >= startSide.number) endSide.number - startSide.number else startSide.number - endSide.number
	val isClockwiseQuickest = if (endSide.number >= startSide.number) delta >= 3 else delta <= 3

	val poses = findPosesBetweenReefSides(startSide, endSide, far, useQuickest, isClockwiseQuickest).toMutableList()

	if (startClose) poses.add(0, FieldConstants.Poses.CLOSE_POSES[startSide.number * 2])
	if (endClose) poses.add(FieldConstants.Poses.CLOSE_POSES[endSide.number * 2])

	val waypoints = PathPlannerPath.waypointsFromPoses(poses)

	val constraints = SwerveConstants.PATH_FOLLOWING_CONSTRAINTS

	return PathPlannerPath(
		waypoints,
		listOf(),
		listOf(PointTowardsZone("Point towards reef", FieldConstants.Poses.REEF_CENTER, 0.0, 1.0)),
		listOf(),
		listOf(),
		constraints,
		IdealStartingState(0.0, poses[0].rotation +
			if ((useQuickest && isClockwiseQuickest) || (!useQuickest && isClockwise)) 90.0.degrees else (-90.0).degrees),
		GoalEndState(0.0, poses.last().rotation +
			if ((useQuickest && isClockwiseQuickest) || (!useQuickest && isClockwise)) 90.0.degrees else (-90.0).degrees),
		false,
	)
}
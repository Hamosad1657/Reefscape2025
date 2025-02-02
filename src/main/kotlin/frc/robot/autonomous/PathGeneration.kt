package frc.robot.autonomous

import com.hamosad1657.lib.ReefSide
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kError
import frc.robot.FieldConstants
import frc.robot.subsystems.swerve.SwerveConstants

// TODO: Make the robot look towards the center throughout the path

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
	val isClockwise = if (endSide.number >= startSide.number) delta >= 3 else delta <= 3

	val numberOfSteps = if (delta > 3) 6-delta else delta

	val poses = MutableList(0) { Pose2d() }
	if ((isClockwise && useQuickest) || (!useQuickest && clockwise) ) {
		for (i in startSide.number downTo(startSide.number-numberOfSteps)) {
			val poseIndex = if (i < 0) (6 + i) * 2 else i * 2
			val nextPoseIndex = if (poseIndex == 0) 11 else poseIndex - 1
			if (far) {
				poses.add(FieldConstants.Poses.FAR_POSES[(poseIndex)])
				if (i != startSide.number-numberOfSteps) poses.add(FieldConstants.Poses.FAR_POSES[nextPoseIndex])
			} else {
				poses.add(FieldConstants.Poses.CLOSE_POSES[poseIndex])
				if (i != startSide.number-numberOfSteps) poses.add(FieldConstants.Poses.CLOSE_POSES[nextPoseIndex])
			}
		}
	} else {
		for (i in startSide.number..startSide.number+numberOfSteps) {
			val poseIndex = if (i > 5) (i - 6) * 2 else i * 2
			if (far) {
				poses.add(FieldConstants.Poses.FAR_POSES[poseIndex])
				if (i != startSide.number+numberOfSteps) poses.add(FieldConstants.Poses.FAR_POSES[poseIndex + 1])
			} else {
				poses.add(FieldConstants.Poses.CLOSE_POSES[poseIndex])
				if (i != startSide.number+numberOfSteps) poses.add(FieldConstants.Poses.CLOSE_POSES[poseIndex + 1])
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
 * @param shouldStartClose - Whether the path should start close to the reef or not.
 * @param shouldEndClose - whether the path should end close to the reef or not.
 * @param far - Whether the path should go around close to the reef or far from it.
 * @param useQuickest - Whether to choose if to go clockwise or counterclockwise automatically or not. If set to false a
 * value for [clockwise] must be given.
 * @param clockwise - If not choosing path automatically, this decides in what direction around the reef the path would go.
 */
fun generatePathAroundReef(
	startSide: ReefSide,
	endSide: ReefSide,
	shouldStartClose: Boolean,
	shouldEndClose: Boolean,
	far: Boolean,
	useQuickest: Boolean,
	clockwise: Boolean = false,
): PathPlannerPath {
	val poses = findPosesBetweenReefSides(startSide, endSide, far, useQuickest, clockwise).toMutableList()
	if (shouldStartClose) poses.add(0, FieldConstants.Poses.CLOSE_POSES[startSide.number * 2])
	if (shouldEndClose) poses.add(FieldConstants.Poses.CLOSE_POSES[endSide.number * 2])
	val waypoints = PathPlannerPath.waypointsFromPoses(poses)
	val constraints = SwerveConstants.PATH_FOLLOWING_CONSTRAINTS
	return PathPlannerPath(waypoints, constraints, null, GoalEndState(0.0, poses.last().rotation))
}
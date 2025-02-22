package frc.robot.commands

import com.hamosad1657.lib.Alert
import com.hamosad1657.lib.Alert.AlertType.ERROR
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.controllers.powerProfile
import com.hamosad1657.lib.units.powerProfile
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.field.FieldConstants
import frc.robot.field.CoralStation
import frc.robot.field.Pipe
import frc.robot.field.ReefSide
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.swerve.getAngleBetweenTranslations
import frc.robot.vision.CoralVision
import kotlin.math.PI
import kotlin.math.absoluteValue

// --- Controller driving command ---

/**
 * Drives regularly with a heading setpoint.
 *
 * @param lJoyYSupplier Supplier for the left joystick's Y value. Up should be positive.
 * @param lJoyXSupplier Supplier for the left joystick's X value. Right should be positive.
 * @param rJoyXSupplier Supplier for the right joystick's X value. Right should be positive.
 */
fun SwerveSubsystem.angularVelocityDriveCommand(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rJoyXSupplier: () -> Double,
	isFieldRelative: Boolean,
	isClosedLoop: () -> Boolean = { false },
	translationMultiplier: () -> Double = { 1.0 },
	rotationMultiplier: () -> Double = { 1.0 },
) = withName("Drive with angular velocity control") {
	run {
		val lJoyY = lJoyYSupplier()
		val lJoyX = lJoyXSupplier()
		val velocity = Translation2d(lJoyX, lJoyY) * SwerveConstants.MAX_SPEED * translationMultiplier()

		val rJoyX = rJoyXSupplier()

		val chassisSpeeds = ChassisSpeeds(
			velocity.y,
			-velocity.x,
			-rJoyX * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs * rotationMultiplier(),
		)

		drive(
			chassisSpeeds,
			isFieldRelative,
			flipForRed = true,
			isClosedLoop(),
		)
	}
}

fun SwerveSubsystem.rotationSetpointDriveCommand(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rotationSetpointSupplier: () -> Rotation2d,
	isFieldRelative: Boolean,
	isClosedLoop: () -> Boolean = { false },
	translationMultiplier: () -> Double = { 1.0 },
) = withName("Drive with rotation setpoint control") {
	run {
		val lJoyY = lJoyYSupplier()
		val lJoyX = lJoyXSupplier()
		val velocity = Translation2d(lJoyX, lJoyY) * SwerveConstants.MAX_SPEED * translationMultiplier()

		val chassisSpeeds = ChassisSpeeds(
			velocity.y,
			-velocity.x,
			SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(currentHeading.radians, rotationSetpointSupplier().radians),
		)

		drive(
			chassisSpeeds,
			isFieldRelative,
			flipForRed = true,
			isClosedLoop(),
		)
	}
}

/** Locks to rotation while staying in place. */
fun SwerveSubsystem.rotateToCommand(rotation: Rotation2d) = withName("Rotate to $rotation") {
	rotationSetpointDriveCommand({ 0.0 }, { 0.0 }, { rotation }, false)
}

fun SwerveSubsystem.rotateToCoralCommand(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rJoyYSupplier: () -> Double,
	rJoyXSupplier: () -> Double,
	isFieldRelative: Boolean,
	isClosedLoop: () -> Boolean = { false },
) = withName("rotateToCoral") {
	run {
		val lJoyY = lJoyYSupplier()
		val lJoyX = lJoyXSupplier()
		val rJoyY = rJoyYSupplier()
		val rJoyX = rJoyXSupplier()
		val velocity = Translation2d(lJoyX, lJoyY) * SwerveConstants.MAX_SPEED
		val chassisSpeeds = if (CoralVision.coralAngleToCenter.radians != 0.0 && rJoyX == 0.0) {
			ChassisSpeeds(
				velocity.y,
				-velocity.x,
				SwerveConstants.CORAL_PID_CONTROLLER.calculate(-CoralVision.coralAngleToCenter.radians, 0.0),
			)
		} else {
			ChassisSpeeds(
				velocity.y,
				-velocity.x,
				-rJoyX
			)
		}

		drive(
			chassisSpeeds,
			isFieldRelative,
			flipForRed = true,
			isClosedLoop(),
		)
	}
}

/**
 * Aims towards a target on the field while staying in place.
 * Uses robot's estimated position on the field.
 * @param target The position of the target on the field relative to the bottom left of the field (standard coordinates).
 */
fun SwerveSubsystem.aimTowardsCommand(target: Translation2d) = withName("Aim towards $target") {
	rotationSetpointDriveCommand({ 0.0 },
		{ 0.0 },
		{ getAngleBetweenTranslations(currentPose.translation, target) },
		false,
	)
}

// --- Path following ---

/**
 * Follows a pathplanner path.
 *
 * @param path - The path to follow.
 * @param flip - Whether to flip the path (for following as red alliance). When set to true, the path
 * is flipped across the x and y of the field, and rotation is rotated by 180 degrees.
 */
fun SwerveSubsystem.followPathCommand(path: PathPlannerPath, flip: Boolean) =
	withName("Follow path ${path.name}") { AutoBuilder.followPath(if (flip) path.flipPath() else path) }

/**
 * Follows a pathplanner path. Resets pose to the start of the path before following.
 *
 * @param path - The path to follow.
 * @param flip - Whether to flip the path (for following as red alliance). When set to true, the path
 * is flipped across the x and y of the field, and rotation is rotated by 180 degrees.
 */
fun SwerveSubsystem.followInitialPathCommand(path: PathPlannerPath, flip: Boolean) =
	withName("Follow initial path ${path.name}") {
		val pathToFollow = if (flip) path.flipPath() else path
		runOnce { resetOdometry(pathToFollow.startingHolonomicPose.get()) } andThen
			followPathCommand(pathToFollow, false)
	}

/** Make sure your project has a navgrid.json in order to know where field obstacles are. */
fun SwerveSubsystem.driveToPoseCommand(target: Pose2d) = withName("drive to $target") {
	AutoBuilder.pathfindToPose(target, SwerveConstants.PATH_FOLLOWING_CONSTRAINTS, 0.0)
}

// --- Pose alignment ---

/** Uses PID on pose estimation to slowly align to [targetPose]. Ends when the estimated pose is within tolerance of
 * the target pose.
 *
 * @param endAutomatically - Whether the command should end by itself when tolerance of the current target pose.
 */
fun SwerveSubsystem.alignToPoseCommand(targetPose: () -> Pose2d, endAutomatically: Boolean) = withName("Align to pose") {
	val linearController = ProfiledPIDController(
		SwerveConstants.POSE_ALIGNMENT_PID_GAINS.kP,
		SwerveConstants.POSE_ALIGNMENT_PID_GAINS.kI,
		SwerveConstants.POSE_ALIGNMENT_PID_GAINS.kD,
		SwerveConstants.POSE_ALIGNMENT_CONSTRAINTS,
	)
	run {
		val currentTargetPose = targetPose()

		val currentDistance = currentPose.translation.getDistance(currentTargetPose.translation)

		val driveVelocityScalar = linearController.calculate(currentDistance, 0.0)
		val driveVelocity = Translation2d(
			driveVelocityScalar,
			currentPose.translation.minus(currentTargetPose.translation).angle
		)

		val chassisSpeeds = ChassisSpeeds(
			driveVelocity.x,
			driveVelocity.y,
			SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(currentHeading.radians, currentTargetPose.rotation.radians),
		)

		drive(
			chassisSpeeds,
			isFieldRelative = true,
			flipForRed = false,
			useClosedLoopDrive = true,
		)
	} until { endAutomatically && (targetPose().translation.getDistance(currentPose.translation) <= SwerveConstants.POSE_ALIGNMENT_TOLERANCE.asMeters) }
}

/**
 * Aligns to a pipe on a reef.
 */
fun SwerveSubsystem.alignToPipeCommand(pipe: () -> Pipe, alliance: Alliance): Command {
	return alignToPoseCommand(
		{
		val targetPose = when (pipe()) {
			Pipe.A -> FieldConstants.Poses.AT_A
			Pipe.B -> FieldConstants.Poses.AT_B
			Pipe.C -> FieldConstants.Poses.AT_C
			Pipe.D -> FieldConstants.Poses.AT_D
			Pipe.E -> FieldConstants.Poses.AT_E
			Pipe.F -> FieldConstants.Poses.AT_F
			Pipe.G -> FieldConstants.Poses.AT_G
			Pipe.H -> FieldConstants.Poses.AT_H
			Pipe.I -> FieldConstants.Poses.AT_I
			Pipe.J -> FieldConstants.Poses.AT_J
			Pipe.K -> FieldConstants.Poses.AT_K
			Pipe.L -> FieldConstants.Poses.AT_L
		}
		if (alliance == Blue) targetPose else FieldConstants.Poses.mirrorPose(targetPose)
		},
		true,
		)
}

/**
 * Aligns to the center of a reef's side.
 */
fun SwerveSubsystem.alignToReefSideCommand(reefSide: () -> ReefSide, alliance: Alliance): Command {
	return alignToPoseCommand(
		{
			val targetPose = when (reefSide()) {
				ReefSide.AB -> FieldConstants.Poses.AT_AB_CENTER
				ReefSide.CD -> FieldConstants.Poses.AT_CD_CENTER
				ReefSide.EF -> FieldConstants.Poses.AT_EF_CENTER
				ReefSide.GH -> FieldConstants.Poses.AT_GH_CENTER
				ReefSide.IJ -> FieldConstants.Poses.AT_IJ_CENTER
				ReefSide.KL -> FieldConstants.Poses.AT_KL_CENTER
			}
			if (alliance == Blue) targetPose else FieldConstants.Poses.mirrorPose(targetPose)
		},
		true,
	)
}

/**
 * Aligns to one of the coral stations.
 */
fun SwerveSubsystem.alignToCoralStationCommand(coralStation: () -> CoralStation, alliance: Alliance): Command {
	return alignToPoseCommand(
		{
			val targetPose = when (coralStation()) {
				CoralStation.KL -> FieldConstants.Poses.KL_CORAL_STATION
				CoralStation.CD -> FieldConstants.Poses.CD_CORAL_STATION
			}
			if (alliance == Blue) targetPose else FieldConstants.Poses.mirrorPose(targetPose)
		},
		true,
	)
}

// --- Other ---

/**
 * Cross locks wheels. Does not end automatically.
 */
fun SwerveSubsystem.crossLockWheelsCommand(): Command = withName("Cross lock wheels") {
	run { crossLockWheels() }
}
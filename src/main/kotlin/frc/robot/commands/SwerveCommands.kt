package frc.robot.commands

import com.hamosad1657.lib.Alert
import com.hamosad1657.lib.Alert.AlertType.ERROR
import com.hamosad1657.lib.Pipe
import com.hamosad1657.lib.Pipe.Companion
import com.hamosad1657.lib.commands.*
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
import frc.robot.FieldConstants
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.swerve.getAngleBetweenTranslations
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
) = withName("Drive with angular velocity control") {
	run {
		val lJoyY = lJoyYSupplier()
		val lJoyX = lJoyXSupplier()
		val velocity = Translation2d(lJoyX, lJoyY) * SwerveConstants.MAX_SPEED

		val rJoyX = rJoyXSupplier()

		val chassisSpeeds = ChassisSpeeds(
			velocity.y,
			-velocity.x,
			-rJoyX * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs,
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
) = withName("Drive with rotation setpoint control") {
	run {
		val lJoyY = lJoyYSupplier()
		val lJoyX = lJoyXSupplier()
		val velocity = Translation2d(lJoyX, lJoyY) * SwerveConstants.MAX_SPEED

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

/**
 * Aims towards a target on the field while staying in place.
 * Uses robot's estimated position on the field.
 * @param target The position of the target on the field relative to the bottom left of the field (standard coordinates).
 */
fun SwerveSubsystem.aimTowardsCommand(target: Translation2d) = withName("Aim towards $target") {
	rotationSetpointDriveCommand({ 0.0 },
		{ 0.0 },
		{ getAngleBetweenTranslations(currentPose.translation, target) },
		false)
}

// --- Path following ---

fun SwerveSubsystem.followPathCommand(path: PathPlannerPath) =
	withName("Follow path $path") { AutoBuilder.followPath(path) }

/** Make sure your project has a navgrid.json in order to know where field obstacles are */
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
	} until { endAutomatically && (
		// TODO: Improve this part
		(targetPose().x - currentPose.x).absoluteValue <= SwerveConstants.POSE_ALIGNMENT_TOLERANCE.asMeters &&
			(targetPose().y - currentPose.y).absoluteValue <= SwerveConstants.POSE_ALIGNMENT_TOLERANCE.asMeters
		)
	}
}

fun SwerveSubsystem.alignToPipe(pipe: Pipe, alliance: Alliance): Command {
	val targetPose = when (pipe) {
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
		else -> Pose2d().also {
			Alert("Invalid pipe alignment request.", ERROR).set(true)
			DriverStation.reportError("Pipe requested to align to of char ${pipe.letter} is not present on the field", true)
			return runOnce {  }
		}
	}
	return alignToPoseCommand({ if (alliance == Blue) targetPose else FieldConstants.Poses.mirrorPose(targetPose) }, true)
}

// --- Other ---

/**
 * Cross locks wheels. Does not end automatically.
 */
fun SwerveSubsystem.crossLockWheelsCommand(): Command = withName("Cross lock wheels") {
	run { crossLockWheels() }
}
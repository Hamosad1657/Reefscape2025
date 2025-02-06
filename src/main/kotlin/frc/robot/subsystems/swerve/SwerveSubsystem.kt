package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo
import com.ctre.phoenix6.swerve.SwerveRequest
import com.hamosad1657.lib.units.MpsSquared
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.FieldConstants
import frc.robot.Robot
import frc.robot.subsystems.swerve.SwerveConstants as Constants
import frc.robot.vision.AprilTagVision.photonAprilTagCamera

object SwerveSubsystem: SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
	::TalonFX,
	::TalonFX,
	::CANcoder,
	Constants.DRIVETRAIN_CONSTANTS,
	Constants.FRONT_LEFT,
	Constants.FRONT_RIGHT,
	Constants.BACK_LEFT,
	Constants.BACK_RIGHT,
), Subsystem, Sendable {
	init {
		SendableRegistry.addLW(this, "SwerveSubsystem", "SwerveSubsystem")
		CommandScheduler.getInstance().registerSubsystem(this)
		configureAutoBuilder()
	}

	val useVisionPoseEstimation = true

	// --- Deriving class member aliases ---

	private inline val pigeon: Pigeon2 get() = super.getPigeon2()
	private inline val currentState: SwerveDriveState get() = super.getState()
	private inline val modulesStates: Array<SwerveModuleState> get() = currentState.ModuleStates

	// --- Robot State Getters ---

	/** Gets the current yaw angle of the robot, as reported by the IMU (CCW positive, not wrapped). */
	val currentHeading: Rotation2d get() = currentPose.rotation

	/** Gets the current velocity (x, y and omega) of the robot. */
	val currentChassisSpeeds: ChassisSpeeds get() = currentState.Speeds

	/** Gets the current pose (position and rotation) of the robot */
	val currentPose: Pose2d get() = state.Pose

	/** Gets the current pitch angle of the robot, as reported by the imu. */
	val currentRobotPitch: Rotation2d get() = Rotation2d.fromDegrees(pigeon.pitch.valueAsDouble)

	val gyroAccel: MpsSquared get() = Translation2d(pigeon.accelerationX.value.baseUnitMagnitude(), pigeon.accelerationY.value.baseUnitMagnitude()).norm

	// --- Drive functions ---

	private val controlRequestFieldRelative = SwerveRequest.FieldCentric()
	private val controlRequestRobotRelative = SwerveRequest.RobotCentric()

	/**
	 * The primary method for controlling the drivebase.
	 *
	 * Takes a chassis speeds and applies it to the modules.
	 *
	 * @param chassisSpeeds - The requested chassis speeds. In field relative mode, chassis speeds are in standard field
	 * relative coordinates, and in robot relative mode x is forward and y is left.
	 * @param isFieldRelative - Drive mode. True for field-relative, false for robot-relative.
	 * @param flipForRed - Whether the chassis speeds should be inverted for field relative driving when the alliance is red.
	 * @param useClosedLoopDrive - Whether to use closed-loop velocity control. Set to true to enable closed-loop.
	 */
	fun drive(
		chassisSpeeds: ChassisSpeeds,
		isFieldRelative: Boolean = true,
		flipForRed: Boolean = true,
		useClosedLoopDrive: Boolean = false,
	) {
		if (isFieldRelative) {
			super.setControl(controlRequestFieldRelative.apply {
				VelocityX = if (Robot.alliance == Blue || !flipForRed) chassisSpeeds.vxMetersPerSecond else -chassisSpeeds.vxMetersPerSecond
				VelocityY = if (Robot.alliance == Blue || !flipForRed) chassisSpeeds.vyMetersPerSecond else -chassisSpeeds.vyMetersPerSecond
				RotationalRate = chassisSpeeds.omegaRadiansPerSecond
				DriveRequestType = if (useClosedLoopDrive) Velocity else OpenLoopVoltage
				SteerRequestType = MotionMagicExpo
			})
		} else {
			super.setControl(controlRequestRobotRelative.apply {
				VelocityX = chassisSpeeds.vxMetersPerSecond
				VelocityY = chassisSpeeds.vyMetersPerSecond
				RotationalRate = chassisSpeeds.omegaRadiansPerSecond
				DriveRequestType = if (useClosedLoopDrive) Velocity else OpenLoopVoltage
				SteerRequestType = MotionMagicExpo
			})
		}
	}

	/**
	 * Set robot relative chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds - Chassis Speeds to set.
	 */
	fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
		drive(chassisSpeeds, isFieldRelative = false, flipForRed = false, useClosedLoopDrive = true)
	}

	private val controlRequestCrossLockWheels = SwerveRequest.SwerveDriveBrake()

	/** Lock the swerve drive (put the modules in a cross-like shape) to prevent it from moving. */
	fun crossLockWheels() {
		super.setControl(controlRequestCrossLockWheels)
	}

	// --- Odometry & Gyro ---

	/** Sets the expected gyroscope angle. */
	fun setGyro(angle: Rotation2d) {
		pigeon.setYaw(angle.degrees)
	}

	/** Sets the gyroscope angle to 0. */
	fun zeroGyro() {
		seedFieldCentric()
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module
	 * positions do not need to be reset when calling this method.
	 *
	 * However, if either gyro angle or module position is reset,
	 * this must be called in order for odometry to keep working.
	 *
	 * @param pose The pose to set the odometry to.
	 */
	fun resetOdometry(pose: Pose2d) {
		resetPose(pose)
		Alert("Reset pose to $pose", kWarning).set(true)
	}

	/** Resets the robot's estimated pose to a given path's starting point. */
	fun resetPoseToPathStart(path: PathPlannerPath, flip: Boolean) {
		resetOdometry(
			if (flip) FieldConstants.Poses.mirrorPose(path.startingHolonomicPose.get())
			else path.startingHolonomicPose.get()
		)
	}

	/** Update the odometry using the detected AprilTag (if any were detected). */
	private val visionFieldWidget = Field2d()

	private fun addVisionMeasurement() {
		if (!photonAprilTagCamera.useOneTagMode) {
			if (photonAprilTagCamera.hasTargets == true) {
				photonAprilTagCamera.estimatedGlobalPose?.let {
					super.addVisionMeasurement(
						it.estimatedPose.toPose2d().let { that ->
							Pose2d(that.x, that.y, currentHeading)
						},
						state.Timestamp,
						photonAprilTagCamera.poseEstimationStdDevs,
					)

					visionFieldWidget.robotPose = it.estimatedPose.toPose2d()
				}
			}
		}
		else {
			if (photonAprilTagCamera.isTagDetected(photonAprilTagCamera.oneTagModeID)) {

				photonAprilTagCamera.estimatedGlobalPose!!.estimatedPose.toPose2d().let {
					super.addVisionMeasurement(
						Pose2d(it.translation, currentHeading),
						state.Timestamp,
						photonAprilTagCamera.poseEstimationStdDevs
					)

					visionFieldWidget.robotPose = it
				}
			}
		}

	}

	// --- PathPlanner ---

	private fun configureAutoBuilder() {
		AutoBuilder.configure(
			::currentPose,
			::resetOdometry,
			::currentChassisSpeeds,
			::setChassisSpeeds,
			Constants.PATH_PLANNER_CONFIG,
			Constants.ROBOT_CONFIG,
			{ false },
			this,
		)
	}

	// --- Periodic ---

	override fun periodic() {
		val allUnreadResults = photonAprilTagCamera.camera.allUnreadResults
		if (allUnreadResults.isNotEmpty()) {
			photonAprilTagCamera.result = allUnreadResults.first()
		}
		if (useVisionPoseEstimation) addVisionMeasurement()

		posesPublisher.set(arrayOf<Pose2d>(currentPose, visionFieldWidget.robotPose))
	}

	// --- Telemetry ---

	private val posesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("poseArray", Pose2d.struct).publish()

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Active command", { currentCommand?.name ?: "none" }, null)

		builder.addDoubleProperty("Current heading deg", { currentHeading.degrees }, null)
		builder.addDoubleArrayProperty("Desired", { state.ModuleTargets.toDoubleArray() }, null)
		builder.addDoubleArrayProperty("Current", { state.ModuleStates.toDoubleArray() }, null)
	}

	private fun Array<SwerveModuleState>.toDoubleArray(): DoubleArray {
		val statesArray = Array(8) { 0.0 }
		for ((index, module) in this.withIndex()) {
			statesArray[index * 2] = module.angle.degrees
			statesArray[index * 2 + 1] = module.speedMetersPerSecond
		}
		return statesArray.toDoubleArray()
	}
}
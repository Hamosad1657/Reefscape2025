package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue.UseClosedLoopSign
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType.Voltage
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType.FusedCANcoder
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Mps
import com.hamosad1657.lib.units.MpsSquared
import com.hamosad1657.lib.units.centimeters
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rps
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import kotlin.math.PI
import frc.robot.RobotMap.Swerve as SwerveMap

object SwerveConstants {
	/** Theoretical free speed (m/s) at 12v applied output. */
	const val MAX_SPEED: Mps = 4.73 // 9.46 according to CTRE ?

	/** Theoretical free rotation speed (rotations/s) at 12v applied output. */
	val MAX_ANGULAR_VELOCITY = 2.0.rps

	/** The distance from the center of the chassis to a center of a module. */
	private val DRIVEBASE_RADIUS = 0.417405.meters

	/** Uses radians. */
	val CHASSIS_ANGLE_PID_CONTROLLER =
		PIDController(1.9, 0.0, 0.0).apply {
			enableContinuousInput(-PI, PI)
		}

	/** Uses radians. */
	val CORAL_PID_CONTROLLER = PIDController(3.5, 0.0, 0.0).apply {
		setTolerance(2.0)
	}

// --- Configs ---

	private val STEER_PID_GAINS: Slot0Configs = Slot0Configs()
		.withKP(60.0).withKI(0.0).withKD(0.0)
		.withKS(0.0).withKV(0.0).withKA(0.0)
		.withStaticFeedforwardSign(UseClosedLoopSign)

	// When using closed-loop control, the drive motor uses the control
	// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
	private val DRIVE_PID_GAINS: Slot0Configs = Slot0Configs()
		.withKP(0.045).withKI(0.0).withKD(0.0)
		.withKS(0.0).withKV(0.130)

	// The closed-loop output type to use for the steer motors
	// This affects the PID/FF gains for the steer motors
	private val STEER_CLOSED_LOOP_OUTPUT_TYPE = Voltage

	// The closed-loop output type to use for the drive motors
	// This affects the PID/FF gains for the drive motors
	private val DRIVE_CLOSED_LOOP_OUTPUT_TYPE = Voltage

	// The type of motor used for the drive motor
	private val DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated

	// The type of motor used for the drive motor
	private val STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated

	// When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
	private val STEER_FEEDBACK_SOURCE = FusedCANcoder

	private val SLIP_CURRENT: Current = Units.Amps.of(120.0)

	// Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
	// Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
	private val INITIAL_DRIVE_CONFIGS = TalonFXConfiguration()
	private val INITIAL_STEER_CONFIGS: TalonFXConfiguration = TalonFXConfiguration()
		.withCurrentLimits(
			CurrentLimitsConfigs() // Swerve azimuth does not require much torque output, so we can set a relatively low
				// stator current limit to help avoid brownouts without impacting performance.
				.withStatorCurrentLimit(Units.Amps.of(60.0))
				.withStatorCurrentLimitEnable(true)
		)
	private val INITIAL_ENCODER_CONFIGS = CANcoderConfiguration()

	// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
	private val PIGEON_CONFIGS: Pigeon2Configuration? = null

	// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
	// This may need to be tuned to your individual robot
	private const val COUPLING_RATIO = 3.5714285714285716

	private const val DRIVE_GEAR_RATIO = 6.746031746031747
	private const val STEER_GEAR_RATIO = 12.8
	private val WHEEL_RADIUS: Distance = Units.Inches.of(2.0)

	private const val SHOULD_INVERT_LEFT_SIDE = false
	private const val SHOULD_INVERT_RIGHT_SIDE = true

	// These are only used for simulation
	private val STEER_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)
	private val DRIVE_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)

	// Simulated voltage necessary to overcome friction
	private val STEER_FRICTION_VOLTAGE: edu.wpi.first.units.measure.Voltage = Units.Volts.of(0.2)
	private val DRIVE_FRICTION_VOLTAGE: edu.wpi.first.units.measure.Voltage = Units.Volts.of(0.2)

	val DRIVETRAIN_CONSTANTS: SwerveDrivetrainConstants = SwerveDrivetrainConstants()
		.withCANBusName(SwerveMap.CANBUS_NAME)
		.withPigeon2Id(SwerveMap.PIGEON_ID)
		.withPigeon2Configs(PIGEON_CONFIGS)

	private val CONSTANT_CREATOR: SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
		SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
			.withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
			.withSteerMotorGearRatio(STEER_GEAR_RATIO)
			.withCouplingGearRatio(COUPLING_RATIO)
			.withWheelRadius(WHEEL_RADIUS)
			.withSteerMotorGains(STEER_PID_GAINS)
			.withDriveMotorGains(DRIVE_PID_GAINS)
			.withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPE)
			.withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPE)
			.withSlipCurrent(SLIP_CURRENT)
			.withSpeedAt12Volts(MAX_SPEED)
			.withDriveMotorType(DRIVE_MOTOR_TYPE)
			.withSteerMotorType(STEER_MOTOR_TYPE)
			.withFeedbackSource(STEER_FEEDBACK_SOURCE)
			.withDriveMotorInitialConfigs(INITIAL_DRIVE_CONFIGS)
			.withSteerMotorInitialConfigs(INITIAL_STEER_CONFIGS)
			.withEncoderInitialConfigs(INITIAL_ENCODER_CONFIGS)
			.withSteerInertia(STEER_INERTIA)
			.withDriveInertia(DRIVE_INERTIA)
			.withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
			.withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)

	// Front Left
	private val FRONT_LEFT_ENCODER_OFFSET: Angle = Units.Degrees.of(67.148)
	private const val FRONT_LEFT_STEER_INVERTED = false
	private const val FRONT_LEFT_ENCODER_INVERTED = false

	private val FRONT_LEFT_X_POS: Distance = Units.Inches.of(11.62)
	private val FRONT_LEFT_Y_POS: Distance = Units.Inches.of(11.62)

	// Front Right
	private val FRONT_RIGHT_ENCODER_OFFSET: Angle = Units.Degrees.of(51.064)
	private const val FRONT_RIGHT_STEER_INVERTED = false
	private const val FRONT_RIGHT_ENCODER_INVERTED = false

	private val FRONT_RIGHT_X_POS: Distance = Units.Inches.of(11.62)
	private val FRONT_RIGHT_Y_POS: Distance = Units.Inches.of(-11.62)

	// Back Left
	private val BACK_LEFT_ENCODER_OFFSET: Angle = Units.Degrees.of(-121.113)
	private const val BACK_LEFT_STEER_INVERTED = false
	private const val BACK_LEFT_ENCODER_INVERTED = false

	private val BACK_LEFT_X_POS: Distance = Units.Inches.of(-11.62)
	private val BACK_LEFT_Y_POS: Distance = Units.Inches.of(11.62)

	// Back Right
	private val BACK_RIGHT_ENCODER_OFFSET: Angle = Units.Degrees.of(-83.145)
	private const val BACK_RIGHT_STEER_INVERTED = false
	private const val BACK_RIGHT_ENCODER_INVERTED = false

	private val BACK_RIGHT_X_POS: Distance = Units.Inches.of(-11.62)
	private val BACK_RIGHT_Y_POS: Distance = Units.Inches.of(-11.62)


	val FRONT_LEFT: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
		CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.FrontLeft.STEER_MOTOR_ID,
			SwerveMap.FrontLeft.DRIVE_MOTOR_ID,
			SwerveMap.FrontLeft.CAN_CODER_ID,
			FRONT_LEFT_ENCODER_OFFSET,
			FRONT_LEFT_X_POS,
			FRONT_LEFT_Y_POS,
			SHOULD_INVERT_LEFT_SIDE,
			FRONT_LEFT_STEER_INVERTED,
			FRONT_LEFT_ENCODER_INVERTED,
		)
	val FRONT_RIGHT: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
		CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.FrontRight.STEER_MOTOR_ID,
			SwerveMap.FrontRight.DRIVE_MOTOR_ID,
			SwerveMap.FrontRight.CAN_CODER_ID,
			FRONT_RIGHT_ENCODER_OFFSET,
			FRONT_RIGHT_X_POS,
			FRONT_RIGHT_Y_POS,
			SHOULD_INVERT_RIGHT_SIDE,
			FRONT_RIGHT_STEER_INVERTED,
			FRONT_RIGHT_ENCODER_INVERTED,
		)
	val BACK_LEFT: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
		CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.BackLeft.STEER_MOTOR_ID,
			SwerveMap.BackLeft.DRIVE_MOTOR_ID,
			SwerveMap.BackLeft.CAN_CODER_ID,
			BACK_LEFT_ENCODER_OFFSET,
			BACK_LEFT_X_POS,
			BACK_LEFT_Y_POS,
			SHOULD_INVERT_LEFT_SIDE,
			BACK_LEFT_STEER_INVERTED,
			BACK_LEFT_ENCODER_INVERTED,
		)
	val BACK_RIGHT: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
		CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.BackRight.STEER_MOTOR_ID,
			SwerveMap.BackRight.DRIVE_MOTOR_ID,
			SwerveMap.BackRight.CAN_CODER_ID,
			BACK_RIGHT_ENCODER_OFFSET,
			BACK_RIGHT_X_POS,
			BACK_RIGHT_Y_POS,
			SHOULD_INVERT_RIGHT_SIDE,
			BACK_RIGHT_STEER_INVERTED,
			BACK_RIGHT_ENCODER_INVERTED,
		)

		// --- PathPlanner ---

		private val PATH_TRANSLATION_CONSTANTS = PIDConstants(4.0, 0.0, 0.0)
		private val PATH_ROTATION_CONSTANTS = PIDConstants(5.0, 0.0, 0.0)

		val PATH_FOLLOWING_CONSTRAINTS = PathConstraints(
			MAX_SPEED,
			MAX_SPEED / 2,
			MAX_ANGULAR_VELOCITY.asRadPs,
			MAX_ANGULAR_VELOCITY.asRadPs * 2,
		)

		val ROBOT_CONFIG = RobotConfig.fromGUISettings()

		val PATH_PLANNER_CONFIG = PPHolonomicDriveController(
			PATH_TRANSLATION_CONSTANTS,
			PATH_ROTATION_CONSTANTS,
		)

	// --- Pose alignment ---

	val POSE_ALIGNMENT_PID_GAINS = PIDGains(
		kP = 0.6,
		kI = 0.1,
		kD = 0.0,
	)

	val POSE_ALIGNMENT_TOLERANCE = 2.2.centimeters

	private const val MAX_POSE_ALIGNMENT_VELOCITY: Mps = 0.6
	private const val MAX_POSE_ALIGNMENT_ACCELERATION: MpsSquared = 0.6

	val POSE_ALIGNMENT_CONSTRAINTS = TrapezoidProfile.Constraints(
		MAX_POSE_ALIGNMENT_VELOCITY,
		MAX_POSE_ALIGNMENT_ACCELERATION,
	)
}
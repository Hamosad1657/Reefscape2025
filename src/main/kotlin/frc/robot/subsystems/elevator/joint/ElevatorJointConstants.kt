package frc.robot.subsystems.elevator.joint

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
import com.ctre.phoenix6.signals.NeutralModeValue.Brake
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.meters
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.RobotMap

object ElevatorJointConstants {
	// --- Elevator constants ---

	val MAIN_ELEVATOR_MOTOR_CONFIGS = TalonFXConfiguration().apply {
		with(MotorOutput) {
			NeutralMode = Brake
			Inverted = Clockwise_Positive
		}
		with(CurrentLimits) {
			SupplyCurrentLimit = 100.0
			SupplyCurrentLimitEnable = true
		}
		with(Feedback) {
			FeedbackRemoteSensorID = RobotMap.ElevatorJoint.HEIGHT_CAN_CODER_ID
			FeedbackSensorSource = RemoteCANcoder
		}
	}

	private val ELEVATOR_CAN_CODER_OFFSET = Rotation2d.fromDegrees(0.0)
	val CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			SensorDirection = SensorDirectionValue.Clockwise_Positive
			MagnetOffset = ELEVATOR_CAN_CODER_OFFSET.rotations
		}
	}

	val ELEVATOR_HEIGHT_PID_GAINS = PIDGains(kP = 0.0)
	const val ELEVATOR_HEIGHT_KG: Volts = 0.0

	val HEIGHT_TOLERANCE: Length = 0.02.meters

	/** For every 1 rotation of the motor, the elevator moves [LENGTH_PER_ROTATION] meters. */
	val LENGTH_PER_ROTATION: Length = 0.0.meters

	val MAX_HEIGHT: Length = 0.0.meters
	val MIN_HEIGHT: Length = 0.0.meters

	val L1_HEIGHT = 0.0.meters
	val L2_HEIGHT = 0.0.meters
	val L3_HEIGHT = 0.0.meters
	val L4_HEIGHT = 0.0.meters

	val LOW_REEF_ALGAE_HEIGHT = 0.0.meters
	val HIGH_REEF_ALGAE_HEIGHT = 0.0.meters
	val PROCESSOR_HEIGHT = 0.0.meters
	val NET_HEIGHT = 0.0.meters

	val INTAKE_HEIGHT = 0.0.meters
	val CORAL_STATION_HEIGHT = 0.0.meters

	// --- Grabber angle constants ---

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kCoast)
		inverted(false)
	}
	val ANGLE_ENCODER_OFFSET = Rotation2d.fromDegrees(0.0)

	/** Works in radians. */
	val ANGLE_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG: Volts = 0.0

	val ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.0)

	val MIN_ANGLE = Rotation2d.fromDegrees(0.0)
	val MAX_ANGLE = Rotation2d.fromDegrees(0.0)

	val L1_ANGLE = Rotation2d.fromDegrees(0.0)
	val L2_ANGLE = Rotation2d.fromDegrees(0.0)
	val L3_ANGLE = Rotation2d.fromDegrees(0.0)
	val L4_ANGLE = Rotation2d.fromDegrees(0.0)

	val REEF_ALGAE_ANGLE = Rotation2d.fromDegrees(0.0)
	val PROCESSOR_ANGLE = Rotation2d.fromDegrees(0.0)
	val NET_ANGLE = Rotation2d.fromDegrees(0.0)

	val INTAKE_ANGLE = Rotation2d.fromDegrees(0.0)
	val CORAL_STATION_ANGLE = Rotation2d.fromDegrees(0.0)
}
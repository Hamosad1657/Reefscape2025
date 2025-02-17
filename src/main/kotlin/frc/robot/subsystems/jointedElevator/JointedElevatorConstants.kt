package frc.robot.subsystems.jointedElevator

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
import com.ctre.phoenix6.signals.NeutralModeValue.Coast
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.RobotMap

object JointedElevatorConstants {
	// --- Elevator constants ---

	val MAIN_ELEVATOR_MOTOR_CONFIGS = TalonFXConfiguration().apply {
		with(MotorOutput) {
			NeutralMode = Coast
			Inverted = Clockwise_Positive
		}
		with(CurrentLimits) {
			SupplyCurrentLimit = 300.0
			SupplyCurrentLimitEnable = true
		}
		with(Feedback) {
			FeedbackRemoteSensorID = RobotMap.JointedElevator.HEIGHT_CAN_CODER_ID
			FeedbackSensorSource = RemoteCANcoder
		}
		with(MotionMagic) {
			MotionMagicAcceleration = 10.0
			MotionMagicCruiseVelocity = 10.0
		}
	}

	private val ELEVATOR_CAN_CODER_OFFSET = Rotation2d.fromRotations(-0.5224609375)
	val HEIGHT_CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			AbsoluteSensorDiscontinuityPoint = 1.0
			SensorDirection = SensorDirectionValue.Clockwise_Positive
			MagnetOffset = ELEVATOR_CAN_CODER_OFFSET.rotations
		}
	}

	val ELEVATOR_PID_GAINS = PIDGains(
		kP = 3.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ELEVATOR_KG: Volts = 0.95

	val ELEVATOR_ROTATION_TOLERANCE: Rotation2d = 0.05.rotations

	val MAX_ELEVATOR_ROTATION = 3.0.rotations
	val MIN_ELEVATOR_ROTATION = 0.0.rotations

	val L1_ELEVATOR_ROTATION = 0.0.rotations
	val L2_ELEVATOR_ROTATION = 0.0.rotations
	val L3_ELEVATOR_ROTATION = 0.0.rotations
	val L4_ELEVATOR_ROTATION = 0.0.rotations

	val LOW_REEF_ALGAE_ELEVATOR_ROTATION = 0.0.rotations
	val HIGH_REEF_ALGAE_ELEVATOR_ROTATION = 0.0.rotations
	val PROCESSOR_ELEVATOR_ROTATION = 0.0.rotations
	val NET_ELEVATOR_ROTATION = 0.0.rotations

	val INTAKE_ELEVATOR_ROTATION = 0.0.rotations
	val CORAL_STATION_ELEVATOR_ROTATION = 0.0.rotations

	// --- Grabber angle constants ---

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kCoast)
		inverted(true)
	}

	private val ANGLE_CAN_CODER_OFFSET = Rotation2d.fromDegrees(-38.8)
	val ANGLE_CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			SensorDirection = SensorDirectionValue.CounterClockwise_Positive
			MagnetOffset = ANGLE_CAN_CODER_OFFSET.rotations
		}
	}

	/** Works in radians. */
	val ANGLE_PID_GAINS = PIDGains(
		kP = 8.5,
		kI = 4.5,
		kD = 0.0,
	)

	val ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.6)

	val MIN_ANGLE = Rotation2d.fromDegrees(-63.4)
	val MAX_ANGLE = Rotation2d.fromDegrees(62.0)

	val REST_ANGLE = Rotation2d.fromDegrees(-15.0)

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
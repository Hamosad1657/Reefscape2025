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
			SupplyCurrentLimit = 40.0
			SupplyCurrentLimitEnable = true
		}
		with(Feedback) {
			FeedbackRemoteSensorID = RobotMap.JointedElevator.HEIGHT_CAN_CODER_ID
			FeedbackSensorSource = RemoteCANcoder
		}
		with(MotionMagic) {
			MotionMagicAcceleration = 16.0
			MotionMagicCruiseVelocity = 16.0
		}
	}

	private val ELEVATOR_CAN_CODER_OFFSET = Rotation2d.fromRotations(0.0)//-0.05029296875)
	val HEIGHT_CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			AbsoluteSensorDiscontinuityPoint = 1.0
			SensorDirection = SensorDirectionValue.Clockwise_Positive
			MagnetOffset = ELEVATOR_CAN_CODER_OFFSET.rotations
		}
	}

	val ELEVATOR_PID_GAINS = PIDGains(
		kP = 8.6,
		kI = 0.0,
		kD = 0.0,
	)
	const val UP_ELEVATOR_KG: Volts = 1.3
	const val DOWN_ELEVATOR_KG: Volts = 0.4

	val ELEVATOR_ROTATION_TOLERANCE: Rotation2d = 0.09.rotations

	val MAX_ELEVATOR_ROTATION = 2.7.rotations
	val MIN_ELEVATOR_ROTATION = 0.0.rotations

	val RESTING_ELEVATOR_ROTATION = 0.0.rotations

	val L1_ELEVATOR_ROTATION = 0.1514.rotations
	val L2_ELEVATOR_ROTATION = 0.742.rotations
	val L3_ELEVATOR_ROTATION = 1.375.rotations
	val L4_ELEVATOR_ROTATION = 2.41333.rotations

	val LOW_REEF_ALGAE_ELEVATOR_ROTATION = 0.982.rotations
	val HIGH_REEF_ALGAE_ELEVATOR_ROTATION = 1.565.rotations
	val PROCESSOR_ELEVATOR_ROTATION = 0.0.rotations
	val NET_ELEVATOR_ROTATION = 0.0.rotations

	val INTAKE_ELEVATOR_ROTATION = 0.0.rotations
	val CORAL_STATION_ELEVATOR_ROTATION = 0.440.rotations

	// --- Grabber angle constants ---

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kCoast)
		inverted(true)
	}

	private val ANGLE_CAN_CODER_OFFSET = Rotation2d.fromDegrees(-97.9)
	val ANGLE_CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			SensorDirection = SensorDirectionValue.CounterClockwise_Positive
			MagnetOffset = ANGLE_CAN_CODER_OFFSET.rotations
		}
	}

	/** Works in radians. */
	val ANGLE_PID_GAINS = PIDGains(
		kP = 7.5,
		kI = 0.0,
		kD = 0.0,
	)

	val ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.6)

	val MIN_ANGLE = Rotation2d.fromDegrees(-69.0)
	val MAX_ANGLE = Rotation2d.fromDegrees(62.0)

	val RESTING_ANGLE = Rotation2d.fromDegrees(-15.0)

	val L1_ANGLE = Rotation2d.fromDegrees(0.0)
	val L2_ANGLE = Rotation2d.fromDegrees(-35.7)
	val L3_ANGLE = Rotation2d.fromDegrees(-36.8)
	val L4_ANGLE = Rotation2d.fromDegrees(-35.5)

	val REEF_ALGAE_ANGLE = Rotation2d.fromDegrees(-66.0)
	val PROCESSOR_ANGLE = Rotation2d.fromDegrees(0.0)
	val NET_ANGLE = Rotation2d.fromDegrees(0.0)

	val INTAKE_ANGLE = Rotation2d.fromDegrees(5.0)
	val CORAL_STATION_ANGLE = Rotation2d.fromDegrees(44.2)
}
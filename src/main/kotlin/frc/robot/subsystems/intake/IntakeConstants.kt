package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.signals.SensorDirectionValue.Clockwise_Positive
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object IntakeConstants {

	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		IdleMode.kBrake
		inverted(false)
		smartCurrentLimit(200)
	}
	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		IdleMode.kBrake
		inverted(false)
	}

	val CAN_CODER_OFFSET = Rotation2d()
	val CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			MagnetOffset = CAN_CODER_OFFSET.rotations
			SensorDirection = Clockwise_Positive
		}
	}

	const val CURRENT_THRESHOLD: Amps = 0

	const val BEAM_BREAK_THRESHOLD: Volts = 10.0

	const val INTAKING_VOLTAGE: Volts = 0.0

	/** Works in radians. */
	val ANGLE_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG = 0.0

	val ANGLE_TOLERANCE = Rotation2d()

	val MAX_ANGLE = Rotation2d()
	val MIN_ANGLE = Rotation2d()

	val RETRACTED_ANGLE = Rotation2d()
	val DEPLOYED_ANGLE = Rotation2d()
}
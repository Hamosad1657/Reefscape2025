package frc.robot.subsystems.intake

import com.revrobotics.spark.config.SparkFlexConfig

object IntakeConstants {
	const val ENCODER_OFFSET = 0.0

	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		inverted(false)

	}
	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		inverted(false)

	}
}
package frc.robot.subsystems.intake

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object IntakeConstants {
	val MAX_ANGLE = Rotation2d()
	val MIN_ANGLE = Rotation2d()

	val RETRACTED_ANGLE = Rotation2d()
	val DEPLOYED_ANGLE = Rotation2d()

	val ANGLE_TOLERANCE = Rotation2d()

	const val CURRENT_THRESHOLD: Amps = 0
	// --- PID ---

	/** Works in rotations. */
	val ANGLE_PID_GAINS = PIDGains(kP = 0.0)
	const val ANGLE_KG = 0.0

	const val INTAKING_VOLTAGE: Volts = 0.0

	// --- Configs ---

	val ENCODER_OFFSET = Rotation2d()

	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		IdleMode.kBrake
		inverted(false)
		smartCurrentLimit(200)
	}
	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		IdleMode.kBrake
		inverted(false)

	}
}
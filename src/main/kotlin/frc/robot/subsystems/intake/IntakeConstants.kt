package frc.robot.subsystems.intake

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object IntakeConstants {

	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		IdleMode.kBrake
		inverted(false)
		smartCurrentLimit(150)
	}
	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		IdleMode.kCoast
		inverted(false)
	}

	val ENCODER_OFFSET = Rotation2d.fromDegrees(304.3)

	val PARALLEL_TO_FLOOR_ANGLE = Rotation2d.fromDegrees(103.0)

	const val BEAM_BREAK_THRESHOLD: Volts = 1.0

	const val INTAKING_VOLTAGE: Volts = 8.5

	const val EJECTING_VOLTAGE: Volts = -11.0

	/** Works in radians. */
	val ANGLE_PID_GAINS = PIDGains(
		kP = 3.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG = 0.65

	val ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0)

	val FALLING_ANGLE_THRESHOLD = Rotation2d.fromDegrees(13.0)

	val MAX_ANGLE = Rotation2d.fromDegrees(105.0)
	val MIN_ANGLE = Rotation2d.fromDegrees(0.0)

	val FEEDING_ANGLE = Rotation2d.fromDegrees(2.0)
	val RESTING_ANGLE = Rotation2d.fromDegrees(35.0)
	val L1_ANGLE = Rotation2d.fromDegrees(46.0)
	val DEPLOYED_ANGLE = Rotation2d.fromDegrees(105.0)
}
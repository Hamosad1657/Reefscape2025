package frc.robot.subsystems.grabber

import com.hamosad1657.lib.math.PIDGains
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object GrabberConstants {
	const val ANGLE_ENCODER_OFFS = 0.0
	const val WHEELS_INTAKE_VOLTAGE = 0.0
	const val WHEELS_EJECT_VOLTAGE = 0.0

	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		inverted(false)
	}

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		inverted(false)
	}

	val ANGLE_PID_CONTROlLER = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	).toPIDController()

	data class GrabberAngles(var angle: Rotation2d) {
		companion object {
			val L1 = 0.0
			val L2 = 0.0
			val L3 = 0.0
			val L4 = 0.0
		}
	}
}
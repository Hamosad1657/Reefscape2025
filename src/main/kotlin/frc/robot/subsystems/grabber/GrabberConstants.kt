package frc.robot.subsystems.grabber

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object GrabberConstants {
	const val ANGLE_ENCODER_OFFS = 0.0
	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		inverted(false)
	}

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		inverted(false)
	}

	val ANGLE_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG = 0.0
	const val ANGLE_TOLERANCE = 0.0

	const val WHEELS_GRAB_VOLTAGE: Volts = 0.0
	const val WHEELS_EJECT_VOLTAGE: Volts = 0.0
	val GRABBING_ANGLE: Rotation2d = 0.0.rotations
	const val GRAB_DURATION_SEC = 0.0
	const val EJECT_DURATION_SEC = 0.0

	const val CURRENT_THRESHOLD: Amps = 0

	val MIN_ANGLE: Rotation2d = 0.0.rotations
	val MAX_ANGLE: Rotation2d = 0.0.rotations

	//--- POSITIONS ---

	data class GrabberAngles(var angle: Rotation2d) {
		companion object {
			val L1 = 0.0
			val L2 = 0.0
			val L3 = 0.0
			val L4 = 0.0
		}
	}
}
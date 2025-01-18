package frc.robot.subsystems.grabber

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object GrabberConstants {
	const val ANGLE_ENCODER_OFFS = 0.0 // This should be a rotation 2d
	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kBrake)
		inverted(false)
		// Add a smart current limit
	}

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kCoast)
		inverted(false)
	}

	val ANGLE_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG = 0.0 // This should be of type Volts
	const val ANGLE_TOLERANCE = 0.0 // This should be a rotation 2d

	const val WHEELS_GRAB_VOLTAGE: Volts = 0.0
	const val WHEELS_EJECT_VOLTAGE: Volts = 0.0
	val GRABBING_ANGLE: Rotation2d = 0.0.rotations // Why is this not in the grabber angles data class?
	// Also I would not call this grab, but something like intake. Please change grab to intake in other places too.

	val MIN_ANGLE: Rotation2d = 0.0.rotations
	val MAX_ANGLE: Rotation2d = 0.0.rotations

	//--- POSITIONS ---

	// This shouldn't be named grabber angles, it represents a single angle for the grabber, therefore it should be singular, a GrabberAngle
	// Needs an angle for the coral station (and intaking as told in a previous comment)
	data class GrabberAngles(var angle: Rotation2d) {
		companion object {
			val L1 = Rotation2d(0.0)
			val L2 = Rotation2d(0.0)
			val L3 = Rotation2d(0.0)
			val L4 = Rotation2d(0.0)
		}
	}
}
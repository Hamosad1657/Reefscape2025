package frc.robot.subsystems.grabber

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object GrabberConstants {
	val WHEEL_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kBrake)
		inverted(false)
		smartCurrentLimit(200)
	}

	val ANGLE_MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kCoast)
		inverted(false)
	}

	val ANGLE_ENCODER_OFFS: Rotation2d = 0.0.rotations


	val ANGLE_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG: Volts = 0.0
	val ANGLE_TOLERANCE = Rotation2d(0.0)

	const val WHEELS_FORWARDS_VOLTAGE: Volts = 0.0
	const val WHEELS_BACKWARDS_VOLTAGE: Volts = 0.0

	val MIN_ANGLE = Rotation2d(0.0)
	val MAX_ANGLE = Rotation2d(0.0)

	//--- POSITIONS ---

	val L1_ANGLE = Rotation2d()
	val L2_ANGLE = Rotation2d()
	val L3_ANGLE = Rotation2d()
	val L4_ANGLE = Rotation2d()

	val INTAKING_ANGLE = Rotation2d()
	val CORAL_STATION_ANGLE = Rotation2d()
}

package frc.robot.subsystems.grabber

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.centimeters
import com.hamosad1657.lib.units.degrees
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object GrabberConstants {
	val MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kBrake)
		inverted(false)
		smartCurrentLimit(200)
	}

	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)

	/** For every 1 rotation of the motor, the corals move [LENGTH_FOR_EACH_ROTATION]. */
	val LENGTH_FOR_EACH_ROTATION: Length = 0.0.centimeters
	val MOTOR_TOLERANCE: Rotation2d = 0.0.degrees

	const val FORWARDS_VOLTAGE: Volts = 0.0
	const val BACKWARDS_VOLTAGE: Volts = 0.0
}

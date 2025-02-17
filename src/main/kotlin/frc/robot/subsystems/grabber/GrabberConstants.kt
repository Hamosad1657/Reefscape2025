package frc.robot.subsystems.grabber

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.centimeters
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.geometry.Rotation2d

object GrabberConstants {
	val MOTOR_CONFIGS = SparkFlexConfig().apply {
		idleMode(kBrake)
		inverted(false)
		smartCurrentLimit(40)
	}

	/** Works in radians. */
	val PID_GAINS = PIDGains(
		kP = 0.17,
		kI = 0.0,
		kD = 0.0,
	)

	val ANGLE_TOLERANCE: Rotation2d = Rotation2d.fromRotations(1.0)

	/** The voltage needed to intake an algae on the reef. */
	const val INTAKE_ALGAE_VOLTAGE: Volts = -12.0

	const val LOAD_FROM_INTAKE_VOLTAGE: Volts = 8.0

	/** The voltage needed to eject a coral to one of the branches of the reef. */
	const val EJECT_CORAL_VOLTAGE: Volts = 12.0

	/** The voltage needed to intake a coral from the coral station. */
	const val INTAKE_FROM_CORAL_STATION_VOLTAGE: Volts = -12.0

	/** The voltage needed to eject an algae to the processor. */
	const val EJECT_ALGAE_TO_PROCESSOR_VOLTAGE: Volts = 12.0

	/** The voltage needed to eject an algae from the grabber to the net. */
	const val EJECT_ALGAE_TO_NET_VOLTAGE: Volts = 12.0

	const val BEAM_BREAK_THRESHOLD: Volts = 2.0
}

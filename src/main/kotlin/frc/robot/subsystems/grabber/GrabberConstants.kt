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
		smartCurrentLimit(40)
	}

	/** Works in radians. */
	val PID_GAINS = PIDGains(
		kP = 0.17,
		kI = 0.0,
		kD = 0.0,
	)

	/** For every 1 rotation of the motor, a coral in the grabber moves [LENGTH_FOR_EACH_ROTATION] inside the grabber. */
	val LENGTH_FOR_EACH_ROTATION: Length = 15.96.centimeters

	val ANGLE_TOLERANCE: Rotation2d = Rotation2d.fromDegrees(20.0)

	/** The voltage needed to move a coral through the intake and out the grabber. */
	const val CORAL_FORWARD_VOLTAGE: Volts = 12.0

	/** The voltage needed to move a coral from the front of the grabber and to the intake. */
	const val CORAL_BACKWARD_VOLTAGE: Volts = -12.0

	/** The voltage needed to intake an algae with the grabber. */
	const val INTAKE_ALGAE_VOLTAGE: Volts = -12.0

	/** The voltage needed to eject an algae from the grabber to the processor. */
	const val EJECT_ALGAE_TO_PROCESSOR_VOLTAGE: Volts = 6.0

	/** The voltage needed to eject an algae from the grabber to the net. */
	const val EJECT_ALGAE_TO_NET_VOLTAGE: Volts = 0.0

	const val BEAM_BREAK_THRESHOLD: Volts = 2.0
}

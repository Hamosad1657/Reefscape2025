package frc.robot.subsystems.climbing

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d

object ClimbingConstants {
	const val CLIMB_VOLTAGE = 0.0
	const val FULLY_CLIMBED_VOLTAGE = 0.0
	const val GRABBING_CAGE_VOLTAGE = 0.0

	val angleTolerance = Rotation2d.fromDegrees(0.0)
	val CANCODER_OFFSET = Rotation2d.fromDegrees(0.0)
	val grabCageAngle = Rotation2d.fromDegrees(0.0)
	val retractedAngle = Rotation2d.fromDegrees(0.0)

	val ANGLE_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)
	const val ANGLE_KG: Volts = 0.0

	val CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			MagnetOffset = CANCODER_OFFSET.rotations
		}
	}
}
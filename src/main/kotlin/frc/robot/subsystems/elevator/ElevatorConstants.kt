package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.meters

object ElevatorConstants {
	val MAIN_MOTOR_CONFIGS = TalonFXConfiguration().apply {
		with(MotorOutput) {
			Inverted = Clockwise_Positive
		}
	}
	val SECONDARY_MOTOR_CONFIGS = TalonFXConfiguration().apply {
		with(MotorOutput) {
			Inverted = Clockwise_Positive
		}
	}

	val HEIGHT_PID_GAINS = PIDGains(kP = 0.0)
	const val HEIGHT_KG = 0.0
	val HEIGHT_TOLERANCE: Length = 0.0.meters
	val MAX_HEIGHT: Length = 0.0.meters

	/** For every 1 rotation of the motor there are 0.0 meters of height are added to the elevator */
	const val ROTATIONS_TO_METERS = 0.0

	val L1_HEIGHT: Length = 0.0.meters
	val L2_HEIGHT: Length = 0.0.meters
	val L3_HEIGHT: Length = 0.0.meters
	val L4_HEIGHT: Length = 0.0.meters
	val CORAL_STATION_HEIGHT: Length = 0.0.meters
}
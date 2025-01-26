package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.meters
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode

object ElevatorConstants {
	val MAIN_MOTOR_CONFIGS = TalonFXConfiguration().apply {
		with(MotorOutput) {
			NeutralModeValue.Brake
			Inverted = Clockwise_Positive
		}
		with(CurrentLimits) {
			SupplyCurrentLimit = 0.0
			SupplyCurrentLimitEnable = true
		}
	}
	val SECONDARY_MOTOR_CONFIGS = TalonFXConfiguration().apply {
		with(MotorOutput) {
			NeutralModeValue.Brake
			Inverted = Clockwise_Positive
		}
		with(CurrentLimits) {
			SupplyCurrentLimit = 0.0
			SupplyCurrentLimitEnable = true
		}
	}
	val CAN_CODER_CONFIGS = CANcoderConfiguration().apply {
		MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
	}

	val HEIGHT_PID_GAINS = PIDGains(kP = 0.0)
	const val HEIGHT_KG: Volts = 0.0

	val HEIGHT_TOLERANCE: Length = 0.02.meters

	/** For every 1 rotation of the motor, the elevator moves [ROTATION_METERS_RATIO] meters. */
	val ROTATION_METERS_RATIO: Length = 0.0.meters

	val MAX_HEIGHT: Length = 0.0.meters

	val L1_HEIGHT = 0.0.meters
	val L2_HEIGHT = 0.0.meters
	val L3_HEIGHT = 0.0.meters
	val L4_HEIGHT = 0.0.meters

	val INTAKE_HEIGHT = 0.0.meters
	val CORAL_STATION_HEIGHT = 0.0.meters
}
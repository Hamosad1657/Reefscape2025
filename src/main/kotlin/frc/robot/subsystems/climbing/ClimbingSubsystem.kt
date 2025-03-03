package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem: SubsystemBase() {
	// --- Motors ---

	private val angleMotor = HaSparkFlex(RobotMap.Climbing.ANGLE_MOTOR_ID).apply {
		IdleMode.kBrake
	}

	// --- Functions ---

	fun setAngleMotorVoltage(voltage: Volts) {
		angleMotor.setVoltage(voltage)
	}
}
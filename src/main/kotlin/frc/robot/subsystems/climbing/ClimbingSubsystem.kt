package frc.robot.subsystems.climbing

import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.degrees
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import kotlin.math.absoluteValue
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem: SubsystemBase() {
	// --- Motors ---

	private val wheelsMotor = HaSparkFlex(RobotMap.Climbing.WHEELS_MOTOR_ID).apply {
		IdleMode.kBrake
	}

	private val angleMotor = HaSparkFlex(RobotMap.Climbing.ANGLE_MOTOR_ID).apply {
		IdleMode.kBrake
	}

	private val angleEncoder = CANcoder(RobotMap.Climbing.CAN_CODER_ID).apply {
		configurator.apply(Constants.CAN_CODER_CONFIGS)
	}

	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController()

	// --- State Getters ---
	private var angleSetpoint: Rotation2d = 0.0.degrees
	private val currentAngle: Rotation2d get() = Rotation2d.fromRotations(angleEncoder.position.valueAsDouble) + Constants.CANCODER_OFFSET
	private val angleError: Rotation2d get() = angleSetpoint - currentAngle
	val isWithinAngleTolerance get() = angleError.degrees.absoluteValue <= Constants.angleTolerance.degrees

	// --- Functions ---
	private fun calculateAngleMotorFF(): Volts = currentAngle.cos * Constants.ANGLE_KG

	private fun isMovingTowardsAngleLimits(output: Double) =
		(output <= 0.0) ||
		(output >= 0.0)

	fun setAngleMotorVoltage(voltage: Volts) {
		angleMotor.setVoltage(voltage)
	}

	fun setWheelsMotorVoltage(voltage: Volts) {
		wheelsMotor.setVoltage(voltage)
	}

	fun setAngleMotorSetpoint(setpoint: Rotation2d) {
		if (setpoint.degrees in 0.0..180.0) {
			angleSetpoint = setpoint
		} else {
			edu.wpi.first.wpilibj
				.Alert("New jointed elevator height setpoint not in motion range. Value not updated.", AlertType.kError)
				.set(true)
			DriverStation.reportWarning("New climbing setpoint of ${setpoint.degrees} degrees is not in the range of motion.", true)
		}
		val output = anglePIDController.calculate(currentAngle.radians, angleSetpoint.radians)
		if (!isMovingTowardsAngleLimits(output)) {
			angleMotor.setVoltage(output + calculateAngleMotorFF())
		} else {
			angleMotor.setVoltage(calculateAngleMotorFF())
		}
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is within angle tolerance", { isWithinAngleTolerance }, null)
	}
}
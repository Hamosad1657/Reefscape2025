package frc.robot.subsystems.grabber

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaSparkMax
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap as Map
import frc.robot.subsystems.grabber.GrabberConstants as Constants

object GrabberSubsystem: SubsystemBase() {
	private val angleMotor = HaSparkFlex(Map.Grabber.ANGLE_MOTOR_ID).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
		angleEncoder
		Constants.ANGLE_PID_GAINS
	}
	private val wheelsMotor = HaSparkMax(Map.Grabber.WHEELS_MOTOR_ID).apply {
		configure(Constants.WHEEL_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}
	private val angleEncoder = DutyCycleEncoder(Map.Grabber.ANGLE_ENCODER_ID)
	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController()

	private val maxAngleLimit = DigitalInput(Map.Grabber.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimit = DigitalInput(Map.Grabber.MIN_ANGLE_LIMIT_CHANNEL)

	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(
		angleEncoder.get() + Constants.ANGLE_ENCODER_OFFS)
	var angleSetpoint = Rotation2d(0.0)
	val angleError get() = abs(angleSetpoint.degrees - currentAngle.degrees) <= Constants.ANGLE_TOLERANCE
	val isAtMaxAngleLimit get() = !maxAngleLimit.get()
	val isAtMinAngleLimit get() = !minAngleLimit.get()

	val isWheelsMotorCurrentAboveThreshold: Boolean get() =
		wheelsMotor.outputCurrent >= Constants.CURRENT_THRESHOLD

	// --- FUNCTIONS---
	fun setAngleSpeed(output: PercentOutput) {
		angleMotor.setVoltage(output)
	}

	fun setWheelsSpeed(output: PercentOutput) {
		wheelsMotor.setVoltage(output)
	}

	fun calculateFF(): Volts {
		return currentAngle.cos * Constants.ANGLE_KG
	}

	fun setWheelsVoltage(voltage: Volts) {
		wheelsMotor.setVoltage(voltage)
	}

	fun getToAngleNoLimits(setpoint: Rotation2d) {
		if (setpoint.rotations <= Constants.MIN_ANGLE.rotations || setpoint.rotations <= Constants.MAX_ANGLE.rotations) {
			Alert("New Grabber angle setpoint not in range. Value not updated", kWarning).set(true)
		}
		angleSetpoint = setpoint
		val output = anglePIDController.calculate(angleSetpoint.degrees)
		angleMotor.setVoltage(output + calculateFF())
	}

	fun getToAngleWithLimits(setpoint: Rotation2d) {
		if ((!isAtMaxAngleLimit && !isAtMinAngleLimit) ||
			(isAtMaxAngleLimit && currentAngle.rotations <= 0.0) ||
			(isAtMinAngleLimit && currentAngle.rotations >= 0.0)) {
			getToAngleNoLimits(setpoint)
		} else {
			angleMotor.setVoltage(calculateFF())
		}
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}

	fun stopWheelsMotor() {
		wheelsMotor.stopMotor()
	}
}
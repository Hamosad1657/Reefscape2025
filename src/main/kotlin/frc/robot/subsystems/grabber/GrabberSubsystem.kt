package frc.robot.subsystems.grabber

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaSparkMax
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.subsystems.grabber.GrabberConstants.GrabberAngle
import kotlin.math.absoluteValue
import frc.robot.RobotMap as Map
import frc.robot.subsystems.grabber.GrabberConstants as Constants

object GrabberSubsystem: SubsystemBase() {

	// --- Components ---

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
	private val beamBreak = DigitalInput(Map.Grabber.BEAM_BREAK_CHANNEL)

	// --- State getters ---

	val isCoralInBeamBreak: Boolean = beamBreak.get() // TODO: check if naturally true or false

	val isAtMaxAngleLimit get() = !maxAngleLimit.get() // TODO: check if naturally true or false
	val isAtMinAngleLimit get() = !minAngleLimit.get() // TODO: check if naturally true or false

	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(
		angleEncoder.get() + Constants.ANGLE_ENCODER_OFFS.degrees)
	var angleSetpoint = Rotation2d(0.0)
	val angleError get() = angleSetpoint.degrees - currentAngle.degrees
	val isAngleWithinTolerance get() = angleError.absoluteValue <= Constants.ANGLE_TOLERANCE.degrees

	// --- Functions ---

	private fun calculateAngleMotorFF(): Volts {
		return currentAngle.cos * Constants.ANGLE_KG
	}

	fun setAngleMotorVoltage(voltage: Volts) {
		angleMotor.setVoltage(voltage)
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}

	fun setWheelsMotorVoltage(voltage: Volts) {
		wheelsMotor.setVoltage(voltage)
	}

	fun stopWheelsMotor() {
		wheelsMotor.stopMotor()
	}

	fun updateAngleControl(setpoint: Rotation2d = angleSetpoint) {
		if (setpoint.rotations <= Constants.MIN_ANGLE.rotations || setpoint.rotations <= Constants.MAX_ANGLE.rotations) {
			Alert("New Grabber angle setpoint not in range. Value not updated", kWarning).set(true)
			DriverStation.reportWarning("Grabber angle request of ${setpoint.degrees} degrees is out of the range of motion", true)
		}
		angleSetpoint = setpoint
		val output = anglePIDController.calculate(currentAngle.degrees, angleSetpoint.degrees)
		if ((!isAtMaxAngleLimit && !isAtMinAngleLimit) ||
			(isAtMaxAngleLimit && output <= 0.0) ||
			(isAtMinAngleLimit && output >= 0.0)) {
			angleMotor.setVoltage(output + calculateAngleMotorFF())
		} else {
			angleMotor.setVoltage(calculateAngleMotorFF())
		}
	}

	// --- Periodic ---

	override fun periodic() {
		if (Robot.isEnabled) updateAngleControl()
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addDoubleProperty("Current angle", { currentAngle.degrees }, null)
		builder.addDoubleProperty("Angle setpoint", { angleSetpoint.degrees }, null)
		builder.addDoubleProperty("Angle error", { angleError }, null)
		builder.addBooleanProperty("Is angle within tolerance", { isAngleWithinTolerance }, null)
		builder.addBooleanProperty("Is at max angle limit", { isAtMaxAngleLimit }, null)
		builder.addBooleanProperty("Is at min angle limit", { isAtMinAngleLimit }, null)

		if (Robot.isTesting) {
			builder.addDoubleProperty("Wheels motor current", { wheelsMotor.outputCurrent }, null)
			builder.addBooleanProperty("Beam break current status", { beamBreak.get() }, null)
		}
	}
}
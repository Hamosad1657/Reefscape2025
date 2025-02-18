package frc.robot.subsystems.intake

import frc.robot.subsystems.intake.IntakeConstants as Constants
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import com.hamosad1657.lib.units.degrees
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeConstants
import kotlin.math.PI

object IntakeSubsystem: SubsystemBase("Intake subsystem") {

	// --- Components ---

	private val wheelMotor = HaSparkFlex(RobotMap.Intake.WHEEL_MOTOR_ID, kBrushless).apply {
		configure(Constants.WHEEL_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val angleMotor = HaSparkFlex(RobotMap.Intake.ANGLE_MOTOR_ID, kBrushless).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val encoder = DutyCycleEncoder(RobotMap.Intake.ENCODER_CHANNEL, 360.0, IntakeConstants.ENCODER_OFFSET.degrees).apply {
		setInverted(false)
	}

	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController().apply {
		enableContinuousInput(0.0, 2 * PI)
	}
	private var angleSetpoint = Rotation2d()

	private val beamBreak = AnalogInput(RobotMap.Intake.BEAM_BREAK_CHANNEL)

	private val maxAngleLimitSwitch = DigitalInput(RobotMap.Intake.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimitSwitch = DigitalInput(RobotMap.Intake.MIN_ANGLE_LIMIT_CHANNEL)

	// --- State getters ---

	val isAtMinAngle: Boolean get() = !minAngleLimitSwitch.get()
	val isAtMaxAngle: Boolean get() =  !maxAngleLimitSwitch.get()

	/** Angle is zero when fully retracted. Angle increases when the intake extends. */
	val currentAngle: Rotation2d get() = Rotation2d.fromDegrees(encoder.get())

	val angleError: Rotation2d get() = angleSetpoint.minus(currentAngle)

	val isWithinAngleTolerance: Boolean get() = currentAngle.absoluteValue <= Constants.ANGLE_TOLERANCE

	val isBeamBreakInterfered: Boolean get() = beamBreak.voltage >= Constants.BEAM_BREAK_THRESHOLD

	// --- Functions ---

	fun setAngleMotorVoltage(voltage: Volts) {
		angleMotor.setVoltage(voltage)
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}

	private fun calculateAngleFF(): Volts {
		return -(currentAngle.minus(Constants.PARALLEL_TO_FLOOR_ANGLE)).cos * Constants.ANGLE_KG
	}

	private fun isMovingTowardsLimits(output: Volts): Boolean = !(
		(!isAtMaxAngle && !isAtMinAngle) ||
			(isAtMaxAngle && output <= 0.0) ||
			(isAtMinAngle && output >= 0.0)
	)

	private fun updateAngleControl(newSetpoint: Rotation2d = angleSetpoint) {
		if (newSetpoint <= Constants.MIN_ANGLE && newSetpoint >= Constants.MAX_ANGLE) {
			Alert("New intake angle setpoint not in range. Value not updated.", kWarning).set(true)
			DriverStation.reportWarning("New angle setpoint ${newSetpoint.degrees} (degrees) is out of range.", true)
		} else angleSetpoint = newSetpoint

		val output = anglePIDController.calculate(currentAngle.radians, angleSetpoint.radians)
		if (!isMovingTowardsLimits(output)) {
			angleMotor.setVoltage(output + calculateAngleFF())
		} else {
			angleMotor.setVoltage(calculateAngleFF())
		}
	}

	fun setAngle(angle: Rotation2d) {
		updateAngleControl(angle)
	}

	fun setWheelMotorVoltage(voltage: Volts) {
		wheelMotor.setVoltage(voltage)
	}

	fun stopWheelMotor() {
		wheelMotor.stopMotor()
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		with(builder) {
			addBooleanProperty("Is at max angle", { isAtMaxAngle }, null)
			addBooleanProperty("Is at min angle", { isAtMinAngle }, null)

			addDoubleProperty("Angle deg", { currentAngle.degrees }, null)
			addDoubleProperty("Angle setpoint deg", { angleSetpoint.degrees }, null)
			addDoubleProperty("Angle error deg", { angleError.absoluteValue.degrees }, null)
			addBooleanProperty("Is angle withing tolerance", { isWithinAngleTolerance }, null)

			addBooleanProperty("Is beam break interfered", { isBeamBreakInterfered }, null)

			if (Robot.isTesting) {
				addDoubleProperty("Wheel motor current Amps", { wheelMotor.outputCurrent }, null)
			}
		}
	}
}
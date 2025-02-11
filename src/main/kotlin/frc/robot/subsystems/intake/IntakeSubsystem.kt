package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.subsystems.intake.IntakeConstants as Constants
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap

object IntakeSubsystem: SubsystemBase("Intake subsystem") {

	// --- Components ---

	private val wheelMotor = HaSparkFlex(RobotMap.Intake.WHEEL_MOTOR_ID, kBrushless).apply {
		configure(Constants.WHEEL_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val angleMotor = HaSparkFlex(RobotMap.Intake.ANGLE_MOTOR_ID, kBrushless).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val canCoder = CANcoder(RobotMap.Intake.CAN_CODER_ID).apply {
		configurator.apply(Constants.CAN_CODER_CONFIGS)
	}

	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController()
	private var angleSetpoint = Rotation2d()

	private val beamBreak = DigitalInput(RobotMap.Intake.BEAM_BREAK_CHANNEL)

	private val maxAngleLimitSwitch = DigitalInput(RobotMap.Intake.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimitSwitch = DigitalInput(RobotMap.Intake.MIN_ANGLE_LIMIT_CHANNEL)

	// --- State getters ---

	val isAtMinAngle: Boolean get() = minAngleLimitSwitch.get()
	val isAtMaxAngle: Boolean get() =  maxAngleLimitSwitch.get()

	/** Angle is zero when fully horizontal. Angle increases when the intake retracts. */
	val currentAngle: Rotation2d get() = canCoder.absolutePosition.value.baseUnitMagnitude().rotations

	val isWithinAngleTolerance: Boolean get() = currentAngle.absoluteValue <= Constants.ANGLE_TOLERANCE

	val isBeamBreakInterfered: Boolean get() = beamBreak.get()

	val isMotorCurrentAboveThreshold: Boolean get() = Constants.CURRENT_THRESHOLD <= wheelMotor.outputCurrent

	// --- Functions ---

	fun setAngleMotorVoltage(voltage: Volts) {
		angleMotor.setVoltage(voltage)
	}

	private fun calculateAngleFF(): Volts {
		return currentAngle.cos * Constants.ANGLE_KG
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

		val output = anglePIDController.calculate(currentAngle.rotations, angleSetpoint.rotations)
		if (!isMovingTowardsLimits(output)) {
			angleMotor.setVoltage(output + calculateAngleFF())
		} else {
			angleMotor.setVoltage(calculateAngleFF())
		}
	}

	private fun setAngle(angle: Rotation2d) {
		updateAngleControl(angle)
	}

	fun setAngleToDeploy() {
		setAngle(Constants.DEPLOYED_ANGLE)
	}

	fun setAngleToRetracted() {
		setAngle(Constants.RETRACTED_ANGLE)
	}

	fun setWheelMotorVoltage(voltage: Volts) {
		wheelMotor.setVoltage(voltage)
	}

	/** Runs the intaking motor so that it will intake a coral and/or drive it towards the elevator. */
	fun runWheelMotor() {
		setWheelMotorVoltage(Constants.INTAKING_VOLTAGE)
	}

	/** Runs the intaking motor in reverse so that it will move a coral in it away from the elevator. */
	fun runWheelMotorReverse() {
		setWheelMotorVoltage(-Constants.INTAKING_VOLTAGE)
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
			addBooleanProperty("Is angle withing tolerance", { isWithinAngleTolerance }, null)

			addBooleanProperty("Is beam break interfered", { isBeamBreakInterfered }, null)

			addBooleanProperty("Is current above threshold", { isMotorCurrentAboveThreshold }, null)

			if (Robot.isTesting) {
				addDoubleProperty("Wheel motor current Amps", { wheelMotor.outputCurrent }, null)
			}
		}
	}
}
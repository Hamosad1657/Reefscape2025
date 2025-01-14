package frc.robot.subsystems.intake

import frc.robot.subsystems.intake.IntakeConstants as Constants
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
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
	private val encoder = DutyCycleEncoder(RobotMap.Intake.ENCODER_ID)
	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController()
	private var angleSetpoint = Rotation2d()

	private val maxAngleLimitSwitch = DigitalInput(RobotMap.Intake.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimitSwitch = DigitalInput(RobotMap.Intake.MIN_ANGLE_LIMIT_CHANNEL)

	// --- State getters ---

	private val isAtMaxAngleLimit: Boolean get() =  maxAngleLimitSwitch.get()
	private val isAtMinAngleLimit: Boolean get() = minAngleLimitSwitch.get()
	/** Angle is zero when fully horizontal. Angle increases when the intake retracts */
	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(encoder.get() + Constants.ENCODER_OFFSET)

	val isMotorCurrentAboveThreshold: Boolean get() = wheelMotor.outputCurrent >= Constants.CURRENT_THRESHOLD

	// --- Functions ---

	private fun calculateIntakeFF(): Volts {
		val angle = currentAngle
		return angle.cos * Constants.ANGLE_KG
	}

	private fun updateAngleControl(newSetpoint: Rotation2d = angleSetpoint) {
		val output = anglePIDController.calculate(currentAngle.rotations, newSetpoint.rotations)
		if ((!isAtMaxAngleLimit && !isAtMinAngleLimit) || (isAtMaxAngleLimit && output <= 0.0) || (isAtMinAngleLimit && output >= 0.0)) {
			angleMotor.setVoltage(output + calculateIntakeFF())
		}
	}

	fun setIntakeAngle(angle: Rotation2d) {
		updateAngleControl(angle)
	}

	fun deployIntake() {
		setIntakeAngle(Constants.DEPLOYED_ANGLE)
	}

	fun retractIntake() {
		setIntakeAngle(Constants.RETRACTED_ANGLE)
	}

	private fun setWheelMotorVoltage(voltage: Volts) {
		wheelMotor.setVoltage(voltage)
	}

	fun stopMotor() {
		wheelMotor.stopMotor()
	}

	/** Runs the intaking motor so that it will intake a coral and/or drive it towards the elevator. */
	private fun runMotor() {
		setWheelMotorVoltage(Constants.INTAKING_VOLTAGE)
	}

	/** Runs the intaking motor in reverse so that it will move a coral in it away from the elevator. */
	private fun runMotorReverse() {
		setWheelMotorVoltage(-Constants.INTAKING_VOLTAGE)
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is at max angle", { isAtMaxAngleLimit }, null)
		builder.addBooleanProperty("Is at min angle", { isAtMinAngleLimit }, null)

		builder.addDoubleProperty("Angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("Angle setpoint deg", { angleSetpoint.degrees }, null)

		builder.addBooleanProperty("Current above threshold", { isMotorCurrentAboveThreshold }, null)

		 if (Robot.isTesting) {
			 builder.addDoubleProperty("Motor current Amps", { wheelMotor.outputCurrent }, null)
		 }

	}
}
package frc.robot.subsystems.elevator.joint

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kError
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap as Map
import kotlin.math.absoluteValue
import frc.robot.subsystems.elevator.joint.ElevatorJointConstants as Constants

object  ElevatorJointSubsystem: SubsystemBase("Elevator") {
	// --- Elevator Components ---

	private val mainElevatorMotor = HaTalonFX(Map.ElevatorJoint.MAIN_MOTOR_ID).apply {
		configurator.apply(Constants.MAIN_ELEVATOR_MOTOR_CONFIGS)
		configPID(Constants.ELEVATOR_HEIGHT_PID_GAINS)
	}
	private val secondaryElevatorMotor = HaTalonFX(Map.ElevatorJoint.SECONDARY_MOTOR_ID).apply {
		Follower(Map.ElevatorJoint.MAIN_MOTOR_ID, true)
	}
	private val heightEncoder = CANcoder(Map.ElevatorJoint.HEIGHT_CAN_CODER_ID).apply {
		configurator.apply(Constants.CAN_CODER_CONFIGS)
	}

	private val maxHeightLimitSwitch = DigitalInput(Map.ElevatorJoint.MAX_HEIGHT_LIMIT_SWITCH_CHANNEL)
	private val minHeightLimitSwitch = DigitalInput(Map.ElevatorJoint.MIN_HEIGHT_LIMIT_SWITCH_CHANNEL)

	// --- Grabber angle components ---

	private val angleMotor = HaSparkFlex(Map.ElevatorJoint.ANGLE_MOTOR_ID).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}
	private val angleEncoder = DutyCycleEncoder(Map.ElevatorJoint.ANGLE_ENCODER_PWM_CHANNEL).apply {
		setInverted(false)
	}
	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController()

	private val maxAngleLimitSwitch = DigitalInput(Map.ElevatorJoint.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimitSwitch = DigitalInput(Map.ElevatorJoint.MIN_ANGLE_LIMIT_CHANNEL)

	// --- State Getters ---

	var isMaintainingState = false

	private var heightSetpoint: Length = Length()

	val isAtMinHeightLimit get() = minHeightLimitSwitch.get()
	val isAtMaxHeightLimit get() = maxHeightLimitSwitch.get()

	val currentHeight: Length get() = Length.fromMeters(mainElevatorMotor.position.value.magnitude() * Constants.LENGTH_PER_ROTATION.asMeters)
	val heightError get() = heightSetpoint - currentHeight
	val isWithinHeightTolerance get() = heightError.meters.absoluteValue <= Constants.HEIGHT_TOLERANCE.asMeters


	private var angleSetpoint: Rotation2d = 0.0.degrees

	val isAtMaxAngleLimit get() = maxAngleLimitSwitch.get()
	val isAtMinAngleLimit get() = minAngleLimitSwitch.get()

	val currentAngle: Rotation2d get() = angleEncoder.get().rotations + Constants.ANGLE_ENCODER_OFFSET
	val angleError: Rotation2d get() = angleSetpoint - currentAngle
	val isWithinAngleTolerance get() = angleError.absoluteValue <= Constants.ANGLE_TOLERANCE

	val isWithinTolerance get() = isWithinAngleTolerance && isWithinHeightTolerance && isMaintainingState

	// --- Functions ---

	fun setElevatorMotorsVoltage(volts: Volts) {
		mainElevatorMotor.setVoltage(volts)
	}

	private var elevatorControlRequest = MotionMagicVoltage(0.0).apply {
		FeedForward = Constants.ELEVATOR_HEIGHT_KG
		Slot = 0
	}
	fun setHeight(newSetpoint: Length = heightError) {
		if (newSetpoint.meters in Constants.MIN_HEIGHT.asMeters..Constants.MAX_HEIGHT.asMeters) {
			heightSetpoint = newSetpoint
		} else {
			Alert("New elevator joint height setpoint not in motion range. Value not updated.", kError).set(true)
			DriverStation.reportWarning("New elevator joint height setpoint of ${newSetpoint.meters} meters is not in the range of motion.", true)
		}
		with(elevatorControlRequest) {
			LimitForwardMotion = isAtMaxHeightLimit
			LimitReverseMotion = isAtMinHeightLimit

			Position = heightSetpoint.asMeters / Constants.LENGTH_PER_ROTATION.asMeters
		}

		mainElevatorMotor.setControl(elevatorControlRequest)
	}


	fun setAngleMotorVoltage(voltage: Volts) {
		angleMotor.setVoltage(voltage)
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}

	private fun calculateAngleMotorFF(): Volts = currentAngle.cos * Constants.ANGLE_KG

	private fun isMovingTowardsAngleLimits(output: Double) = !((!isAtMaxAngleLimit && !isAtMinAngleLimit) ||
			(isAtMaxAngleLimit && output <= 0.0) ||
				(isAtMinAngleLimit && output >= 0.0)
		)

	fun updateAngleControl(newSetpoint: Rotation2d = angleSetpoint) {
		if (newSetpoint > Constants.MAX_ANGLE || newSetpoint < Constants.MIN_ANGLE) {
			Alert("New elevator joint angle setpoint not in range. Value not updated", kWarning).set(true)
			DriverStation.reportWarning("Elevator angle request of ${newSetpoint.degrees} degrees is out of the range of motion", true)
		} else angleSetpoint = newSetpoint

		val output = anglePIDController.calculate(currentAngle.radians, angleSetpoint.radians)
		if (!isMovingTowardsAngleLimits(output)) {
			angleMotor.setVoltage(output + calculateAngleMotorFF())
		} else {
			angleMotor.setVoltage(calculateAngleMotorFF())
		}
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		with(builder) {
			addBooleanProperty("Is at min height", { isAtMinHeightLimit }, null)
			addBooleanProperty("Is at max height", { isAtMaxHeightLimit }, null)
			addDoubleProperty("Height error cm", { heightError.asCentimeters }, null)
			addBooleanProperty("Is in height tolerance", { isWithinHeightTolerance }, null)
			addDoubleProperty("Elevator height Meters", { currentHeight.asMeters }, null)
			addDoubleProperty("Elevator setpoint Meters", { heightSetpoint.asMeters }, null)

			addDoubleProperty("Current angle deg", { currentAngle.degrees }, null)
			addDoubleProperty("Angle setpoint deg", { angleSetpoint.degrees }, null)
			addDoubleProperty("Angle error deg", { angleError.degrees }, null)
			addBooleanProperty("Is angle within tolerance", { isWithinAngleTolerance }, null)
			addBooleanProperty("Is at max angle limit", { isAtMaxAngleLimit }, null)
			addBooleanProperty("Is at min angle limit", { isAtMinAngleLimit }, null)

			addBooleanProperty("Is within Tolerance", { isWithinTolerance }, null)

			if (Robot.isTesting) {
				addDoubleProperty("Elevator Motor current Amps", { mainElevatorMotor.supplyCurrent.value.baseUnitMagnitude() }, null)

				addDoubleProperty("Angle motor current Amps", { angleMotor.outputCurrent }, null)
			}
		}
	}
}
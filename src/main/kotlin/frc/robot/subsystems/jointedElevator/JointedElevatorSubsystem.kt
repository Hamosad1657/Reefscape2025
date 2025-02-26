package frc.robot.subsystems.jointedElevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import com.hamosad1657.lib.units.degrees
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kError
import edu.wpi.first.wpilibj.Alert.AlertType.kWarning
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import kotlin.math.absoluteValue
import frc.robot.RobotMap as Map
import frc.robot.subsystems.jointedElevator.JointedElevatorConstants as Constants

object  JointedElevatorSubsystem: SubsystemBase("Jointed elevator") {
	// --- Elevator Components ---

	val elevatorRotationEncoder = CANcoder(Map.JointedElevator.HEIGHT_CAN_CODER_ID).apply {
		configurator.apply(Constants.HEIGHT_CAN_CODER_CONFIGS)
		configurator.setPosition(0.0)
	}

	private val mainElevatorMotor = HaTalonFX(Map.JointedElevator.MAIN_HEIGHT_MOTOR_ID).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.MAIN_ELEVATOR_MOTOR_CONFIGS)
		configPID(Constants.ELEVATOR_PID_GAINS)
	}
	private val secondaryElevatorMotor = HaTalonFX(Map.JointedElevator.SECONDARY_HEIGHT_MOTOR_ID).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.MAIN_ELEVATOR_MOTOR_CONFIGS)
		configPID(Constants.ELEVATOR_PID_GAINS)
	}

	private val maxHeightLimitSwitch = DigitalInput(Map.JointedElevator.MAX_HEIGHT_LIMIT_SWITCH_CHANNEL)
	private val minHeightLimitSwitch = DigitalInput(Map.JointedElevator.MIN_HEIGHT_LIMIT_SWITCH_CHANNEL)

	// --- Grabber angle components ---

	private val angleMotor = HaSparkFlex(Map.JointedElevator.ANGLE_MOTOR_ID).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}
	private val angleEncoder = CANcoder(Map.JointedElevator.ANGLE_CAN_CODER_ID).apply {
		configurator.apply(Constants.ANGLE_CAN_CODER_CONFIGS)
	}
	private val anglePIDController = Constants.ANGLE_PID_GAINS.toPIDController()

	private val maxAngleLimitSwitch = DigitalInput(Map.JointedElevator.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimitSwitch = DigitalInput(Map.JointedElevator.MIN_ANGLE_LIMIT_CHANNEL)

	// --- State Getters ---

	var isMaintainingState = false

	private var elevatorRotationSetpoint: Rotation2d = Rotation2d.fromRotations(0.0)

	val isAtMinHeightLimit get() = !minHeightLimitSwitch.get()
	val isAtMaxHeightLimit get() = !maxHeightLimitSwitch.get()

	val currentElevatorRotation: Rotation2d get() = Rotation2d.fromRotations(elevatorRotationEncoder.position.value.magnitude())
	val ElevatorHeightError get() = Rotation2d.fromRotations(elevatorRotationSetpoint.rotations - currentElevatorRotation.rotations)
	val isElevatorWithinTolerance get() = ElevatorHeightError.absoluteValue <= Constants.ELEVATOR_ROTATION_TOLERANCE


	private var angleSetpoint: Rotation2d = 0.0.degrees

	val isAtMaxAngleLimit get() = !maxAngleLimitSwitch.get()
	val isAtMinAngleLimit get() = !minAngleLimitSwitch.get()

	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(angleEncoder.absolutePosition.value.magnitude())
	val angleError: Rotation2d get() = angleSetpoint - currentAngle
	val isWithinAngleTolerance get() = angleError.absoluteValue <= Constants.ANGLE_TOLERANCE

	val isWithinTolerance get() = isWithinAngleTolerance && isElevatorWithinTolerance && isMaintainingState

	// --- Functions ---

	fun setElevatorMotorsVoltage(volts: Volts) {
		if (volts !in 0.0..1.3) {
			mainElevatorMotor.setVoltage(volts)
			secondaryElevatorMotor.setVoltage(volts)
		}
	}

	private var elevatorControlRequest = MotionMagicVoltage(0.0).apply {
		Slot = 0
	}
	fun updateElevatorRotationControl(newSetpoint: Rotation2d = elevatorRotationSetpoint) {
		if (newSetpoint.rotations in Constants.MIN_ELEVATOR_ROTATION.rotations..Constants.MAX_ELEVATOR_ROTATION.rotations) {
			elevatorRotationSetpoint = newSetpoint
		} else {
			Alert("New jointed elevator height setpoint not in motion range. Value not updated.", kError).set(true)
			DriverStation.reportWarning("New jointed elevator height setpoint of ${newSetpoint.rotations} rotations is not in the range of motion.", true)
		}
		with(elevatorControlRequest) {
			Position = elevatorRotationSetpoint.rotations

			LimitForwardMotion = isAtMaxHeightLimit
			LimitReverseMotion = isAtMinHeightLimit

			FeedForward = if (elevatorRotationSetpoint >= currentElevatorRotation) Constants.UP_ELEVATOR_KG else Constants.DOWN_ELEVATOR_KG
		}

		mainElevatorMotor.setControl(elevatorControlRequest)
		secondaryElevatorMotor.setControl(Follower(Map.JointedElevator.MAIN_HEIGHT_MOTOR_ID, false))
	}


	fun setAngleMotorVoltage(voltage: Volts) {
		if ((voltage >= 0.0 && !isAtMaxAngleLimit) || (voltage <= 0.0 && !isAtMinAngleLimit)) angleMotor.setVoltage(voltage)
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}

	private fun isMovingTowardsAngleLimits(output: Double) = !((!isAtMaxAngleLimit && !isAtMinAngleLimit) ||
			(isAtMaxAngleLimit && output <= 0.0) ||
				(isAtMinAngleLimit && output >= 0.0)
		)

	fun updateAngleControl(newSetpoint: Rotation2d = angleSetpoint) {
		if (newSetpoint > Constants.MAX_ANGLE || newSetpoint < Constants.MIN_ANGLE) {
			Alert("New jointed elevator angle setpoint not in range. Value not updated", kWarning).set(true)
			DriverStation.reportWarning("jointed elevator angle request of ${newSetpoint.degrees} degrees is out of the range of motion", true)
		} else angleSetpoint = newSetpoint

		val output = anglePIDController.calculate(currentAngle.radians, angleSetpoint.radians)
		if (!isMovingTowardsAngleLimits(output)) {
			angleMotor.setVoltage(output)
		} else {
			angleMotor.stopMotor()
		}
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		with(builder) {
			addBooleanProperty("Is at min height", { isAtMinHeightLimit }, null)
			addBooleanProperty("Is at max height", { isAtMaxHeightLimit }, null)
			addDoubleProperty("Height error rotations", { ElevatorHeightError.rotations }, null)
			addBooleanProperty("Is in height tolerance", { isElevatorWithinTolerance }, null)
			addDoubleProperty("Elevator rotation rotations", { currentElevatorRotation.rotations }, null)
			addDoubleProperty("Elevator absolute rotation rotations", { elevatorRotationEncoder.absolutePosition.value.magnitude() }, null)
			addDoubleProperty("Elevator setpoint rotations", { elevatorRotationSetpoint.rotations }, null)

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
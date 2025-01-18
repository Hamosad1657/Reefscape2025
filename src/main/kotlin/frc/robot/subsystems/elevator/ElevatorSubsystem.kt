package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.meters
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import kotlin.math.absoluteValue
import frc.robot.subsystems.elevator.ElevatorConstants as Constants

object ElevatorSubsystem: SubsystemBase() {

	// --- Limit Switches ---

	private val maxHeightLimitSwitch = DigitalInput(RobotMap.Elevator.MAX_HEIGHT_LIMIT_SWITCH_ID)
	private val minHeightLimitSwitch = DigitalInput(RobotMap.Elevator.MIN_HEIGHT_LIMIT_SWITCH_ID)
	// --- Motors ---

	private val mainMotor = HaTalonFX(RobotMap.Elevator.MAIN_MOTOR_ID).apply {
		configurator.apply(Constants.MAIN_MOTOR_CONFIGS)
		configPID(Constants.HEIGHT_PID_GAINS)
	}
	private val secondaryMotor = HaTalonFX(RobotMap.Elevator.SECONDARY_MOTOR_ID).apply {
		configurator.apply(Constants.SECONDARY_MOTOR_CONFIGS)
		Follower(RobotMap.Elevator.MAIN_MOTOR_ID, false)
	}

	// --- Encoders ---

	private val heightEncoder = CANcoder(RobotMap.Elevator.CANCODER_ID)

	// --- State Getters ---
	private var currentSetpoint: Length = 0.0.meters

	private val isAtMaxHeight get() = maxHeightLimitSwitch.get()
	private val isAtMinHeight get() = minHeightLimitSwitch.get()

	val currentHeight: Length get() = (heightEncoder.position.valueAsDouble * Constants.ROTATIONS_TO_METERS).meters
	val isWithinTolerance get() = (currentSetpoint.meters - currentHeight.meters).absoluteValue < Constants.HEIGHT_TOLERANCE.meters


	// --- Functions ---
	fun setElevatorMotorsVolts(volts: Volts) {
		mainMotor.setVoltage(volts)
	}

	var mainMotorControlRequest = MotionMagicVoltage(0.0).apply {
		FeedForward = Constants.HEIGHT_KG
	}
	fun setHeight(newSetpoint: Length) {
		currentSetpoint = newSetpoint
		mainMotorControlRequest.Position = newSetpoint.meters / Constants.ROTATIONS_TO_METERS
		mainMotorControlRequest.Slot = 0
		mainMotor.setControl(mainMotorControlRequest)
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is at max height", { isAtMaxHeight }, null)
		builder.addBooleanProperty("Is at min height", { isAtMinHeight }, null)
		builder.addBooleanProperty("Is at tolerance", { isWithinTolerance }, null)
		builder.addDoubleProperty("elevator height meters", { currentHeight.asMeters }, null)
		builder.addDoubleProperty("elevator setpoint meters", { currentSetpoint.asMeters }, null)
	}
}
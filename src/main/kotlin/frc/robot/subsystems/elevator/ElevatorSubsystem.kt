package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.meters
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kError
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap
import kotlin.math.absoluteValue
import frc.robot.subsystems.elevator.ElevatorConstants as Constants

object ElevatorSubsystem: SubsystemBase() {
	// --- Components ---

	private val mainMotor = HaTalonFX(RobotMap.Elevator.MAIN_MOTOR_ID).apply {
		configurator.apply(Constants.MAIN_MOTOR_CONFIGS)
		configPID(Constants.HEIGHT_PID_GAINS)
	}
	private val secondaryMotor = HaTalonFX(RobotMap.Elevator.SECONDARY_MOTOR_ID).apply {
		configurator.apply(Constants.SECONDARY_MOTOR_CONFIGS)
		Follower(RobotMap.Elevator.MAIN_MOTOR_ID, false)
	}
	private val maxHeightLimitSwitch = DigitalInput(RobotMap.Elevator.MAX_HEIGHT_LIMIT_SWITCH_CHANNEL)
	private val minHeightLimitSwitch = DigitalInput(RobotMap.Elevator.MIN_HEIGHT_LIMIT_SWITCH_CHANNEL)

	private val heightEncoder = CANcoder(RobotMap.Elevator.CAN_CODER_ID).apply {
		configurator.apply(Constants.CAN_CODER_CONFIGS)
	}

	// --- State Getters ---

	private var currentSetpoint: Length = 0.0.meters

	val isAtMinHeight get() = minHeightLimitSwitch.get()
	val isAtMaxHeight get() = maxHeightLimitSwitch.get()

	val currentHeight: Length get() = Length.fromMeters(heightEncoder.positionSinceBoot.valueAsDouble * Constants.ROTATION_METERS_RATIO.asMeters)
	val isWithinHeightTolerance get() = (currentSetpoint.meters - currentHeight.meters).absoluteValue <= Constants.HEIGHT_TOLERANCE.asMeters


	// --- Functions ---

	fun setElevatorMotorsVoltage(volts: Volts) {
		mainMotor.setVoltage(volts)
	}

	private var elevatorControlRequest = MotionMagicVoltage(0.0).apply {
		FeedForward = Constants.HEIGHT_KG
	}
	fun setHeight(newSetpoint: Length) {
		if (newSetpoint.meters in 0.0..Constants.MAX_HEIGHT.asMeters) {
			currentSetpoint = newSetpoint
		} else {
			Alert("New elevator setpoint not in motion range!", kError).set(true)
			DriverStation.reportWarning("New elevator setpoint of ${newSetpoint.meters} meters is not in the elevator motion range.", true)
		}
		with(elevatorControlRequest) {
			Position = if ((isAtMaxHeight && (currentHeight < newSetpoint)) || (isAtMinHeight && (newSetpoint < currentHeight))) {
				currentHeight.asMeters / Constants.ROTATION_METERS_RATIO.asMeters
			} else {
				newSetpoint.asMeters / Constants.ROTATION_METERS_RATIO.asMeters
			}
			Slot = 0
		}

		mainMotor.setControl(elevatorControlRequest)
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is at min height", { isAtMinHeight }, null)
		builder.addBooleanProperty("Is at max height", { isAtMaxHeight }, null)

		builder.addBooleanProperty("Is in tolerance", { isWithinHeightTolerance }, null)

		builder.addDoubleProperty("Elevator height Meters", { currentHeight.asMeters }, null)
		builder.addDoubleProperty("Elevator setpoint Meters", { currentSetpoint.asMeters }, null)

		if (Robot.isTesting) {
			builder.addDoubleProperty("Motor current Amps", { mainMotor.supplyCurrent.value.baseUnitMagnitude() }, null)
		}
	}
}
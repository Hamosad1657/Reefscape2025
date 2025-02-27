package frc.robot.subsystems.grabber

import com.hamosad1657.lib.motors.HaSparkMax
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.compareTo
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rotations
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap as Map
import frc.robot.subsystems.grabber.GrabberConstants as Constants

object GrabberSubsystem: SubsystemBase() {
	// --- Components ---

	private val motor = HaSparkMax(Map.Grabber.MOTOR_ID).apply {
		configure(Constants.MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val beamBreak = AnalogInput(Map.Grabber.BEAM_BREAK_CHANNEL)

	private val PIDController = Constants.PID_GAINS.toPIDController()

	// --- State getters ---

	val isBeamBreakInterfered: Boolean get() = beamBreak.voltage >= Constants.BEAM_BREAK_THRESHOLD

	var isUsingPIDControl = false
		private set

	private var setpoint: Rotation2d = 0.0.degrees

	val currentAngle: Rotation2d get() = motor.encoder.position.rotations
	val pidError: Rotation2d get() = setpoint.minus(currentAngle)
	val isInTolerance: Boolean get() = pidError.absoluteValue <= Constants.ANGLE_TOLERANCE && isUsingPIDControl

	// --- Functions ---

	fun setMotorVoltage(voltage: Volts) {
		isUsingPIDControl = false
		motor.setVoltage(voltage)
	}

	/** Sets the setpoint of the motor relative to where it is now. */
	fun setMotorSetpoint(newSetpoint: Rotation2d = setpoint) {
		isUsingPIDControl = true
		setpoint = Rotation2d.fromRotations(
			currentAngle.rotations + newSetpoint.rotations
		)
		updateMotorPIDControl()
	}

	fun updateMotorPIDControl() {
		motor.setVoltage(PIDController.calculate(currentAngle.radians, setpoint.radians))
	}

	fun stopMotor() {
		isUsingPIDControl = false
		motor.stopMotor()
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		with(builder) {
			if (Robot.isTesting) {
				addBooleanProperty("Is coral in beam break", { isBeamBreakInterfered }, null)

				addBooleanProperty("Is using PID control", { isUsingPIDControl }, null)
				addDoubleProperty("Current angle", { currentAngle.degrees }, null)
				addDoubleProperty("Setpoint deg", { setpoint.degrees }, null)
				addDoubleProperty("Angle error deg", { pidError.degrees }, null)
				addBooleanProperty("Is in setpoint tolerance", { isInTolerance }, null)

				addDoubleProperty("Motor current Amps", { motor.outputCurrent }, null)
				addDoubleProperty("Beambreak voltage", { beamBreak.voltage }, null)
			}
		}
	}
}
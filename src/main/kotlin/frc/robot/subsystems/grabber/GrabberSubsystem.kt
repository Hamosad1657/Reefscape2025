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
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap as Map
import frc.robot.subsystems.grabber.GrabberConstants as Constants

object GrabberSubsystem: SubsystemBase() {
	// --- Components ---

	private val motor = HaSparkMax(Map.Grabber.MOTOR_ID).apply {
		configure(Constants.MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val beamBreak = DigitalInput(Map.Grabber.BEAM_BREAK_CHANNEL)

	private val PIDController = Constants.PID_GAINS.toPIDController()

	// --- State getters ---

	val isBeamBreakInterfered: Boolean get() = beamBreak.get()
	val currentAngle: Rotation2d get() = motor.encoder.position.rotations
	var setpoint: Rotation2d = 0.0.degrees

	val isInTolerance: Boolean get() = (setpoint - currentAngle).absoluteValue <= Constants.MOTOR_TOLERANCE

	// --- Functions ---

	fun setMotorVoltage(voltage: Volts) {
		motor.setVoltage(voltage)
	}

	/** Sets the setpoint of the motor relative to where it is now. */
	fun setMotorSetpoint(lengthSetpoint: Length) {
		setpoint = Rotation2d.fromRotations(
			currentAngle.rotations + (lengthSetpoint.asCentimeters / Constants.LENGTH_FOR_EACH_ROTATION.asCentimeters)
		)
	}

	fun updateMotorPIDControl() {
		motor.setVoltage(PIDController.calculate(currentAngle.radians, setpoint.radians))
	}

	fun stopMotor() {
		motor.stopMotor()
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is coral in beam break", { isBeamBreakInterfered }, null)
		
		builder.addDoubleProperty("Setpoint deg", { setpoint.degrees }, null)
		builder.addDoubleProperty("Current angle", { currentAngle.degrees }, null)
		builder.addBooleanProperty("Is in setpoint tolerance", { isInTolerance }, null)

		if (Robot.isTesting) {
			builder.addDoubleProperty("Motor current Amps", { motor.outputCurrent }, null)
		}
	}
}
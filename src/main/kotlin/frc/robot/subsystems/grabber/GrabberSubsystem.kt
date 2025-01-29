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

	private val PIDController = Constants.PID_GAINS.toPIDController()

	// --- Components ---

	private val motor = HaSparkMax(Map.Grabber.WHEELS_MOTOR_ID).apply {
		configure(Constants.MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val beamBreak = DigitalInput(Map.Grabber.BEAM_BREAK_CHANNEL)

	// --- State getters ---

	val isBeamBreakInterfered: Boolean = beamBreak.get()
	val currentAngle: Rotation2d = motor.encoder.position.rotations
	private var setpoint: Rotation2d = 0.0.degrees
	val isInTolerance: Boolean get() = (setpoint - currentAngle).absoluteValue <= Constants.MOTOR_TOLERANCE

	// --- Functions ---

	fun setMotorVoltage(voltage: Volts) {
		motor.setVoltage(voltage)
	}

	fun setMotorSetpoint(lengthSetpoint: Length) {
		setpoint = Rotation2d.fromRotations(lengthSetpoint.asCentimeters / Constants.LENGTH_FOR_EACH_ROTATION.asCentimeters)
	}

	fun updateMotorPIDControl() {
		motor.setVoltage(MathUtil.clamp(PIDController.calculate(currentAngle.rotations, currentAngle.rotations + setpoint.rotations), -1.5, 1.5))
	}

	fun stopMotor() {
		motor.stopMotor()
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
			builder.addBooleanProperty("Beam break current status", { beamBreak.get() }, null)

		if (Robot.isTesting) {
			builder.addDoubleProperty("Motor current Amps", { motor.outputCurrent }, null)
		}
	}
}
package frc.robot.subsystems.grabber

import com.hamosad1657.lib.motors.HaSparkMax
import com.hamosad1657.lib.units.Volts
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap as Map
import frc.robot.subsystems.grabber.GrabberConstants as Constants

object GrabberSubsystem: SubsystemBase() {

	private val PIDController = PIDController(Constants.pidGains.kP, Constants.pidGains.kI, Constants.pidGains.kD)

	// --- Components ---

	private val motor = HaSparkMax(Map.Grabber.WHEELS_MOTOR_ID).apply {
		configure(Constants.WHEEL_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val beamBreak = DigitalInput(Map.Grabber.BEAM_BREAK_CHANNEL)

	// --- State getters ---

	val isCoralInBeamBreak: Boolean = beamBreak.get()

	// --- Functions ---

	fun setMotorVoltage(voltage: Volts) {
		motor.setVoltage(voltage)
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
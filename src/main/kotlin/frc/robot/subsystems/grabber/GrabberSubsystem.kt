package frc.robot.subsystems.grabber

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaSparkMax
import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap as Map
import frc.robot.subsystems.grabber.GrabberConstants as Constants

object GrabberSubsystem: SubsystemBase() {
	private val angleMotor = HaSparkFlex(Map.Grabber.ANGLE_MOTOR_ID).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
		angleEncoder
	}
	private val wheelsMotor = HaSparkMax(Map.Grabber.WHEELS_MOTOR_ID).apply {
		configure(Constants.WHEEL_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val angleEncoder = DutyCycleEncoder(Map.Grabber.ANGLE_ENCODER_ID)

	private val maxAngleLimit = DigitalInput(Map.Grabber.MAX_ANGLE_LIMIT_CHANNEL)
	private val minAngleLimit = DigitalInput(Map.Grabber.MIN_ANGLE_LIMIT_CHANNEL)

	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(
		angleEncoder.get() + Constants.ANGLE_ENCODER_OFFS)
	val isAtMaxAngleLimit get() = !maxAngleLimit.get()
	val isAtMinAngleLimit get() = !minAngleLimit.get()
}
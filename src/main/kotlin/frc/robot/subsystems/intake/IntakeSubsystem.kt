package frc.robot.subsystems.intake

import com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
import com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters
import frc.robot.subsystems.intake.IntakeConstants as Constants
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object IntakeSubsystem: SubsystemBase("Intake subsystem") {
	private val wheelMotor = SparkFlex(RobotMap.Intake.WHEEL_MOTOR_ID, kBrushless).apply {
		configure(Constants.WHEEL_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}

	private val angleMotor = SparkFlex(RobotMap.Intake.ANGLE_MOTOR_ID, kBrushless).apply {
		configure(Constants.ANGLE_MOTOR_CONFIGS, kResetSafeParameters, kPersistParameters)
	}
	private val encoder = DutyCycleEncoder(RobotMap.Intake.ENCODER_ID)

	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(encoder.get()+Constants.ENCODER_OFFSET)

}
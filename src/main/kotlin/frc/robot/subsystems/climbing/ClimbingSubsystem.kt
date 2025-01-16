package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaSparkFlex
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem: SubsystemBase() {
	// --- Motors ---

	private val climbingMotor = HaSparkFlex(RobotMap.Climbing.CLIMBING_MOTOR_ID).apply {
		IdleMode.kBrake
	}
	
	// --- Limit Switches ---

	private val limitSwitch = DigitalInput(RobotMap.Climbing.LIMIT_SWITCH_ID)

	// --- State Getters ---

	private val isAtMaxClimb get() = limitSwitch.get()

	// --- Functions ---

	fun climb() {
		if (!isAtMaxClimb) {
			climbingMotor.setVoltage(Constants.CLIMB_VOLTAGE)
		} else {
			climbingMotor.stopMotor()
		}
	}

	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is at max climb", { isAtMaxClimb }, null)
	}
}
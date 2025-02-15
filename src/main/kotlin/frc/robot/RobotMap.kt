package frc.robot

object RobotMap {
	const val DRIVER_A_CONTROLLER_PORT = 0
	const val DRIVER_B_CONTROLLER_PORT = 1
	const val TESTING_CONTROLLER_PORT = 5

	object Swerve {
		const val CANBUS_NAME = "SwerveBus"
		const val PIGEON_ID = 0

		object FrontLeft {
			const val DRIVE_MOTOR_ID = 15
			const val STEER_MOTOR_ID = 5
			const val CAN_CODER_ID = 6
		}

		object FrontRight {
			const val DRIVE_MOTOR_ID = 16
			const val STEER_MOTOR_ID = 7
			const val CAN_CODER_ID = 8
		}

		object BackLeft {
			const val DRIVE_MOTOR_ID = 17
			const val STEER_MOTOR_ID = 9
			const val CAN_CODER_ID = 10
		}

		object BackRight {
			const val DRIVE_MOTOR_ID = 18
			const val STEER_MOTOR_ID = 11
			const val CAN_CODER_ID = 12
		}
	}

	object JointedElevator {
		const val MAIN_HEIGHT_MOTOR_ID = 0
		const val SECONDARY_HEIGHT_MOTOR_ID = 0
		const val MAX_HEIGHT_LIMIT_SWITCH_CHANNEL = 0
		const val MIN_HEIGHT_LIMIT_SWITCH_CHANNEL = 0
		const val HEIGHT_CAN_CODER_ID = 0

		const val ANGLE_MOTOR_ID = 0
		const val ANGLE_ENCODER_CHANNEL = 0
		const val MAX_ANGLE_LIMIT_CHANNEL = 0
		const val MIN_ANGLE_LIMIT_CHANNEL = 0
	}
}
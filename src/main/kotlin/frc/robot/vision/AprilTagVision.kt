package frc.robot.vision

import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.vision.apriltags.RobotPoseStdDevs
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import frc.robot.vision.PhotonAprilTagCamera.AprilTagsStdDevs
import kotlin.math.PI

object AprilTagVision {
	private val robotToCameraTranslation: Translation3d = Translation3d(0.352, 0.1715, 0.312)
	private val robotToCameraRotation: Rotation3d = Rotation3d(0.0, 0.0, 0.0)

	private val stdDevs
		get() = AprilTagsStdDevs(
			oneTag = RobotPoseStdDevs(0.9, 0.9, 0.95),
			twoTagsAuto = RobotPoseStdDevs(0.5, 0.5, 0.95),
			twoTagsTeleop = RobotPoseStdDevs(0.35, 0.35, 0.95),
		)

	val photonAprilTagCamera = PhotonAprilTagCamera(
		"OrangePi-Cam",
		robotToCameraTranslation,
		robotToCameraRotation,
		5.0.meters,
		5.0.meters,
		0.2,
		stdDevs
	)
}



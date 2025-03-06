package frc.robot.vision

import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.vision.apriltags.PhotonAprilTagCamera
import com.hamosad1657.lib.vision.apriltags.RobotPoseStdDevs
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import com.hamosad1657.lib.vision.apriltags.PhotonAprilTagCamera.AprilTagsStdDevs

object AprilTagVision {
	private val robotToCameraTranslation: Translation3d = Translation3d(0.35, 0.175, 0.312)
	private val robotToCameraRotation: Rotation3d = Rotation3d(0.0, 0.0, 0.0)

	private val stdDevs
		get() = AprilTagsStdDevs(
			oneTag = RobotPoseStdDevs(1.0, 1.15, 0.95),
			twoTagsAuto = RobotPoseStdDevs(0.9, 1.0, 0.95),
			twoTagsTeleop = RobotPoseStdDevs(0.9, 1.0, 0.95),
		)

	val photonAprilTagCamera = PhotonAprilTagCamera(
		"AprilTag-Camera",
		robotToCameraTranslation,
		robotToCameraRotation,
		5.0.meters,
		5.0.meters,
		0.2,
		stdDevs
	)
}



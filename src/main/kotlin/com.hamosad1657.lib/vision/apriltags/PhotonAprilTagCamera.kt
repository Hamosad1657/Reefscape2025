package frc.robot.vision

import com.ctre.phoenix6.hardware.Pigeon2
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.vision.apriltags.RobotPoseStdDevs
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields.kDefaultField
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import frc.robot.Robot
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.jvm.optionals.getOrNull
import kotlin.math.absoluteValue



class PhotonAprilTagCamera(
	cameraName: String,
	robotToCameraTranslation: Translation3d,
	robotToCameraRotation: Rotation3d,
	private val maxRange: Length,
	private val maxTagTrustingDistance: Length,
	private val maxAmbiguity: Double,
	private val stdDevs: AprilTagsStdDevs
	) {
	val camera: PhotonCamera = PhotonCamera(cameraName)
	private val isConnected get() = camera.isConnected
	var result: PhotonPipelineResult? = null
	private val allTargets get() = result?.targets
	val hasTargets get() = result?.hasTargets()
	private val bestTarget: PhotonTrackedTarget? get() = result?.bestTarget
	private val targetID get() = bestTarget?.fiducialId
	var useOneTagMode = false
	var oneTagModeID = 0
	val poseEstimationStdDevs
		get() = if (result?.targets?.size == 1) {
			stdDevs.oneTag
		} else if (Robot.isAutonomous) {
			stdDevs.twoTagsAuto
		} else {
			stdDevs.twoTagsTeleop
		}

	//Find StdDevs
//	var lastEstimatedPose: Pose3d? = null
//	var lastPigeonAngle: Rotation2d = 0.0.degrees
//	private val xErrorsList: MutableList<Double> = emptyList<Double>().toMutableList()
//	private val yErrorsList: MutableList<Double> = emptyList<Double>().toMutableList()
//	private val rotationErrorsList: MutableList<Double> = emptyList<Double>().toMutableList()
//	private val timer = edu.wpi.first.wpilibj.Timer()
//	private val xStdDevs = calculateGeneralStdDevs(xErrorsList.toDoubleArray())
//	private val yStdDevs = calculateGeneralStdDevs(yErrorsList.toDoubleArray())
//	private val rotationStdDevs = calculateGeneralStdDevs(rotationErrorsList.toDoubleArray())

	private val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(kDefaultField)

	private val robotToCamera = Transform3d(
		robotToCameraTranslation,
		robotToCameraRotation
	)

	private var photonPoseEstimator: PhotonPoseEstimator =
		PhotonPoseEstimator(aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera)

	val isInVisionRange: Boolean
		get() {
			if (hasTargets == true) {
				for (i in camera.allUnreadResults[0].targets) {
					(idAndHeightMap[targetID])?.let {
						if (calculateRangeWithTargetHeight(it) < maxRange.asMeters) {
							return true
						}
					}
				}
			}
			return false
		}
	fun getTargetWithID(tagID: Int): PhotonTrackedTarget? {
		if (hasTargets == true) {
			for (i in allTargets!!) {
				if (targetID == tagID) {
					return i
				}
			}
		}
		return null
	}

	val isInRange: Boolean
		get() {
			if (hasTargets == true) {
				val robotToTagDistance =
					(idAndHeightMap[bestTarget!!.fiducialId])?.let { calculateRangeWithTargetHeight(it) }
				return robotToTagDistance!! < maxTagTrustingDistance.asMeters
			}
			else {
				return false
			}
		}

	fun isTagDetected(tagId: Int): Boolean {
		if (hasTargets == true) {
			for (i in allTargets!!) {
				if (targetID == tagId) {
					return true
				}
			}
		}
		return false
	}

	private fun calculateRangeWithTargetHeight(targetHeightMeters: Double): Double {
		return PhotonUtils.calculateDistanceToTargetMeters(
			robotToCamera.z,
			targetHeightMeters,
			robotToCamera.rotation.y,
			bestTarget?.let { Units.degreesToRadians(it.pitch) } ?: 0.0,
			)
	}

//	private fun calculateGeneralStdDevs(numArray: DoubleArray): Double {
//		var sum = 0.0
//		var standardDeviation = 0.0
//
//		for (num in numArray) {
//			sum += num
//		}
//
//		val mean = sum / 10
//
//		for (num in numArray) {
//			standardDeviation += Math.pow(num - mean, 2.0)
//		}
//
//		return Math.sqrt(standardDeviation / 10)
//	}

//	fun calculatePositionStdDevs(pigeon: Pigeon2): RobotPoseStdDevs {
//		estimatedGlobalPose?.let {
//			lastEstimatedPose?.let {
//				timer.start()
//				if (timer.hasElapsed(0.2)) {
//					rotationErrorsList.add(((estimatedGlobalPose!!.estimatedPose.rotation.angle - it.rotation.angle).absoluteValue / 0.02) - ((pigeon.yaw.valueAsDouble - lastPigeonAngle.degrees).absoluteValue / 0.02))
//					xErrorsList.add(((estimatedGlobalPose!!.estimatedPose.x - it.x).absoluteValue / 0.02) - pigeon.accelerationX.valueAsDouble)
//					yErrorsList.add(((estimatedGlobalPose!!.estimatedPose.y - it.y).absoluteValue / 0.02) - pigeon.accelerationY.valueAsDouble)
//
//					if (xErrorsList.size > 10) {
//						xErrorsList.removeAt(10)
//					}
//					if (yErrorsList.size > 10) {
//						yErrorsList.removeAt(10)
//					}
//					if (rotationErrorsList.size > 10) {
//						rotationErrorsList.removeAt(10)
//					}
//
//					timer.reset()
//					timer.stop()
//				}
//			}
//		}
//		return RobotPoseStdDevs(xStdDevs, yStdDevs, rotationStdDevs)
//	}

	fun setModeToOneTag(wantedID: Int) {
		useOneTagMode = true
		oneTagModeID = wantedID
	}

	fun setModeToGlobal() {
		useOneTagMode = false
	}

	val estimatedGlobalPose: EstimatedRobotPose?
		get() {
			if (!isConnected) return null
			result ?: return null
			if (hasTargets == null || bestTarget == null) return null
			if (bestTarget!!.poseAmbiguity > maxAmbiguity) return null
			return photonPoseEstimator.update(result).getOrNull()
		}
	private val idAndHeightMap: HashMap<Int, Double> = HashMap<Int, Double>().apply {
		put(1, 1.35255)
		put(2, 1.35255)
		put(3, 1.165225)
		put(4, 1.7526)
		put(5, 1.7526)
		put(6, 0.174625)
		put(7, 0.174625)
		put(8, 0.174625)
		put(9, 0.174625)
		put(10, 0.174625)
		put(11, 0.174625)
		put(12, 1.35255)
		put(13, 1.35255)
		put(14, 1.7526)
		put(15, 1.7526)
		put(16, 1.165225)
		put(17, 0.174625)
		put(18, 0.174625)
		put(19, 0.174625)
		put(20, 0.174625)
		put(21, 0.174625)
		put(22, 0.174625)
	}

	data class AprilTagsStdDevs(
		val oneTag: RobotPoseStdDevs,
		val twoTagsAuto: RobotPoseStdDevs,
		val twoTagsTeleop: RobotPoseStdDevs,
		)
}



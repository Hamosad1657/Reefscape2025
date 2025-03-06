package frc.robot.vision

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.vision.HaPhotonCamera
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object CoralVision {
	val coralCamera = HaPhotonCamera("Coral-Camera")

	private var result: PhotonPipelineResult? = null
	private val allTargets get() = result?.targets
	val hasTargets get() = result?.hasTargets()
	val bestTarget: PhotonTrackedTarget? get() = result?.bestTarget
	val coralYaw: Rotation2d get() = Rotation2d.fromDegrees((bestTarget?.getYaw() ?: 0.0.degrees) as Double)
	val coralPitch: Rotation2d get() = Rotation2d.fromDegrees((bestTarget?.getPitch() ?: 0.0.degrees) as Double)
	val coralSkew: Rotation2d get() = Rotation2d.fromDegrees((bestTarget?.getSkew() ?: 0.0.degrees) as Double)
}
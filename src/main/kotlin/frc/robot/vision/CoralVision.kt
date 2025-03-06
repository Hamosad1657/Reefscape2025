package frc.robot.vision

import com.hamosad1657.lib.leds.Degrees
import com.hamosad1657.lib.vision.HaPhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object CoralVision {
	val coralCamera = HaPhotonCamera("Coral-Camera")

	private var result: PhotonPipelineResult? = null
	private val allTargets get() = result?.targets
	val hasTargets get() = result?.hasTargets()
	val bestTarget: PhotonTrackedTarget? get() = result?.bestTarget
	val coralAngleToCenter: Double get() = bestTarget?.getYaw() ?: 0.0
}
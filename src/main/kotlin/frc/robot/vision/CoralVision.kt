package frc.robot.vision

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.vision.HaPhotonCamera
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.absoluteValue

object CoralVision {
	val CORAL_INTAKING_YAW = Rotation2d.fromDegrees(-13.6)
	val CORAL_INTAKING_YAW_TOLERANCE = Rotation2d.fromDegrees(3.5)

	val coralCamera = HaPhotonCamera("Coral-Camera")

	var result: PhotonPipelineResult? = null

	private val allTargets get() = result?.targets
	val hasTargets get() = result?.hasTargets() ?: false
	val bestTarget: PhotonTrackedTarget? get() = result?.bestTarget

	val coralYaw: Rotation2d get() = Rotation2d.fromDegrees((bestTarget?.getYaw() ?: 0.0))
	val isCoralYawWithinTolerance: Boolean get() = (CORAL_INTAKING_YAW.degrees - coralYaw.degrees).absoluteValue.let {
		it < CORAL_INTAKING_YAW_TOLERANCE.degrees && it != 0.0
	}

	val coralPitch: Rotation2d get() = Rotation2d.fromDegrees((bestTarget?.getPitch() ?: 0.0.degrees) as Double)

	val coralSkew: Rotation2d get() = Rotation2d.fromDegrees((bestTarget?.getSkew() ?: 0.0.degrees) as Double)
}
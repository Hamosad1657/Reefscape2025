package frc.robot.field

import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

object FieldConstants {
	val fieldLength = Length.fromInches(690.876)
	val fieldWidth = Length.fromInches(317)

	object Poses {
		/** Mirrors a pose on the field in both axes, and rotates it by 180.0 degrees */
		fun mirrorPose(pose: Pose2d) = Pose2d(
			fieldLength.asMeters - pose.x,
			fieldWidth.asMeters - pose.y,
			pose.rotation + 180.0.degrees,
		)

		val AT_PROCESSOR = Pose2d(5.98744, 0.58700,Rotation2d.fromDegrees(-90.0))

		val REEF_CENTER = Translation2d(4.48934, 4.02592)

		val AB_CLOSE = Pose2d(2.82951, 4.02592, Rotation2d.fromDegrees(0.0))
		val AB_FAR = Pose2d(2.04684, 4.02592, Rotation2d.fromDegrees(0.0))

		val AB_CD_CLOSE = Pose2d(3.05189, 3.196, Rotation2d.fromDegrees(30.0))
		val AB_CD_FAR = Pose2d(2.37407, 2.80467, Rotation2d.fromDegrees(30.0))

		val CD_CLOSE = Pose2d(3.65942, 2.58847, Rotation2d.fromDegrees(60.0))
		val CD_FAR = Pose2d(3.26809, 1.91065, Rotation2d.fromDegrees(60.0))

		val CD_EF_CLOSE = Pose2d(4.48934, 2.36609, Rotation2d.fromDegrees(90.0))
		val CD_EF_FAR = Pose2d(4.48934, 1.58342, Rotation2d.fromDegrees(90.0))

		val EF_CLOSE = Pose2d(5.31925, 2.58847, Rotation2d.fromDegrees(120.0))
		val EF_FAR = Pose2d(5.71059, 1.91065, Rotation2d.fromDegrees(120.0))

		val EF_GH_CLOSE = Pose2d(5.92679, 3.19600, Rotation2d.fromDegrees(150.0))
		val EF_GH_FAR = Pose2d(6.60460, 2.80467, Rotation2d.fromDegrees(150.0))

		val GH_CLOSE = Pose2d(6.14916, 4.02592, Rotation2d.fromDegrees(180.0))
		val GH_FAR = Pose2d(6.93184, 4.02592, Rotation2d.fromDegrees(180.0))

		val GH_IJ_CLOSE = Pose2d(5.92679, 4.85583, Rotation2d.fromDegrees(-150.0))
		val GH_IJ_FAR = Pose2d(6.60460, 5.24717, Rotation2d.fromDegrees(-150.0))

		val IJ_CLOSE = Pose2d(5.31925, 5.46337, Rotation2d.fromDegrees(-120.0))
		val IJ_FAR = Pose2d(5.71059, 6.14118, Rotation2d.fromDegrees(-120.0))

		val IJ_KL_CLOSE = Pose2d(4.48934, 5.68574, Rotation2d.fromDegrees(-90.0))
		val IJ_KL_FAR = Pose2d(4.48934, 6.46842, Rotation2d.fromDegrees(-90.0))

		val KL_CLOSE = Pose2d(3.65942, 5.46337, Rotation2d.fromDegrees(-60.0))
		val KL_FAR = Pose2d(3.26809, 6.14118, Rotation2d.fromDegrees(-60.0))

		val KL_AB_CLOSE = Pose2d(3.05189, 4.85583, Rotation2d.fromDegrees(-30.0))
		val KL_AB_FAR = Pose2d(2.37407, 5.24717, Rotation2d.fromDegrees(-30.0))

		val KL_CORAL_STATION = Pose2d(1.11524, 7.03288, Rotation2d.fromDegrees(125.989))
		val CD_CORAL_STATION = Pose2d(1.11514, 1.01899, Rotation2d.fromDegrees(-125.989))

		val AT_A = Pose2d(3.21060, 4.19023, Rotation2d.fromDegrees(0.0))
		val AT_B = Pose2d(3.21060, 3.86161, Rotation2d.fromDegrees(0.0))
		val AT_C = Pose2d(3.70767, 3.00065, Rotation2d.fromDegrees(60.0))
		val AT_D = Pose2d(3.99226, 2.83634, Rotation2d.fromDegrees(60.0))
		val AT_E = Pose2d(4.98641, 2.83634, Rotation2d.fromDegrees(120.0))
		val AT_F = Pose2d(5.27100, 3.00065, Rotation2d.fromDegrees(120.0))
		val AT_G = Pose2d(5.76807, 3.86161, Rotation2d.fromDegrees(180.0))
		val AT_H = Pose2d(5.76807, 4.19023, Rotation2d.fromDegrees(180.0))
		val AT_I = Pose2d(5.27100, 5.05118, Rotation2d.fromDegrees(-120.0))
		val AT_J = Pose2d(4.98641, 5.21549, Rotation2d.fromDegrees(-120.0))
		val AT_K = Pose2d(3.99226, 5.21549, Rotation2d.fromDegrees(-60.0))
		val AT_L = Pose2d(3.70767, 5.05118, Rotation2d.fromDegrees(-60.0))

		val AT_AB_LEFT = Pose2d(3.21060, 4.43458, Rotation2d.fromDegrees(0.0))
		val AT_AB_RIGHT = Pose2d(3.21060, 3.61726, Rotation2d.fromDegrees(0.0))
		val AT_CD_LEFT = Pose2d(3.49606, 3.12283, Rotation2d.fromDegrees(60.0))
		val AT_CD_RIGHT = Pose2d(4.20388, 2.71417, Rotation2d.fromDegrees(60.0))
		val AT_EF_LEFT = Pose2d(4.77480, 2.71417, Rotation2d.fromDegrees(120.0))
		val AT_EF_RIGHT = Pose2d(5.48261, 3.12283, Rotation2d.fromDegrees(120.0))
		val AT_GH_LEFT = Pose2d(5.76807, 3.61726, Rotation2d.fromDegrees(180.0))
		val AT_GH_RIGHT = Pose2d(5.76807, 4.43458, Rotation2d.fromDegrees(180.0))
		val AT_IJ_LEFT = Pose2d(5.48261, 4.92901, Rotation2d.fromDegrees(-120.0))
		val AT_IJ_RIGHT = Pose2d(4.77480, 5.33767, Rotation2d.fromDegrees(-120.0))
		val AT_KL_LEFT = Pose2d(4.20388, 5.33767, Rotation2d.fromDegrees(-60.0))
		val AT_KL_RIGHT = Pose2d(3.49606, 4.92901, Rotation2d.fromDegrees(-60.0))


		val AT_AB_CENTER = Pose2d(3.21060, 4.02592, Rotation2d.fromDegrees(0.0))
		val AT_CD_CENTER = Pose2d(3.84997, 2.91850, Rotation2d.fromDegrees(60.0))
		val AT_EF_CENTER = Pose2d(5.12871, 2.91850, Rotation2d.fromDegrees(120.0))
		val AT_GH_CENTER = Pose2d(5.76807, 4.02592, Rotation2d.fromDegrees(180.0))
		val AT_IJ_CENTER = Pose2d(5.12871, 5.13334, Rotation2d.fromDegrees(-120.0))
		val AT_KL_CENTER = Pose2d(3.84997, 5.13334, Rotation2d.fromDegrees(-60.0))

		//TODO: Find position (except for the angle)
		val AT_NET_POSITION = Pose2d(7.34414, 6.04308, Rotation2d.fromDegrees(0.0))

		val CLOSE_POSES = listOf(
			AB_CLOSE,
			AB_CD_CLOSE,
			CD_CLOSE,
			CD_EF_CLOSE,
			EF_CLOSE,
			EF_GH_CLOSE,
			GH_CLOSE,
			GH_IJ_CLOSE,
			IJ_CLOSE,
			IJ_KL_CLOSE,
			KL_CLOSE,
			KL_AB_CLOSE,
		)
		val FAR_POSES = listOf(
			AB_FAR,
			AB_CD_FAR,
			CD_FAR,
			CD_EF_FAR,
			EF_FAR,
			EF_GH_FAR,
			GH_FAR,
			GH_IJ_FAR,
			IJ_FAR,
			IJ_KL_FAR,
			KL_FAR,
			KL_AB_FAR,
		)
	}
}
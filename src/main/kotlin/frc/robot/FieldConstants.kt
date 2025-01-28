package frc.robot

import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d

object FieldConstants {
	object Poses {
		val AB_CLOSE = Pose2d(2.92934, 4.02592, Rotation2d.fromDegrees(0.0))
		val AB_FAR = Pose2d(2.04684, 4.02592, Rotation2d.fromDegrees(0.0))

		val AB_CD_CLOSE = Pose2d(3.13834, 3.246, Rotation2d.fromDegrees(30.0))
		val AB_CD_FAR = Pose2d(2.37407, 2.80467, Rotation2d.fromDegrees(30.0))

		val CD_CLOSE = Pose2d(3.70934, 2.67492, Rotation2d.fromDegrees(60.0))
		val CD_FAR = Pose2d(3.26809, 1.91065, Rotation2d.fromDegrees(60.0))

		val CD_EF_CLOSE = Pose2d(4.48934, 2.46592, Rotation2d.fromDegrees(90.0))
		val CD_EF_FAR = Pose2d(4.48934, 1.58342, Rotation2d.fromDegrees(90.0))

		val EF_CLOSE = Pose2d(5.26934, 2.67492, Rotation2d.fromDegrees(120.0))
		val EF_FAR = Pose2d(5.71059, 1.91065, Rotation2d.fromDegrees(120.0))

		val EF_GH_CLOSE = Pose2d(5.84034, 3.24592, Rotation2d.fromDegrees(150.0))
		val EF_GH_FAR = Pose2d(6.60460, 2.80467, Rotation2d.fromDegrees(150.0))

		val GH_CLOSE = Pose2d(6.04934, 4.02592, Rotation2d.fromDegrees(180.0))
		val GH_FAR = Pose2d(6.93184, 4.02592, Rotation2d.fromDegrees(180.0))

		val GH_IJ_CLOSE = Pose2d(5.84034, 4.80592, Rotation2d.fromDegrees(-150.0))
		val GH_IJ_FAR = Pose2d(6.60460, 5.24717, Rotation2d.fromDegrees(-150.0))

		val IJ_CLOSE = Pose2d(5.26934, 5.37692, Rotation2d.fromDegrees(-120.0))
		val IJ_FAR = Pose2d(5.71059, 6.14118, Rotation2d.fromDegrees(-120.0))

		val IJ_KL_CLOSE = Pose2d(4.48934, 5.58592, Rotation2d.fromDegrees(-90.0))
		val IJ_KL_FAR = Pose2d(4.48934, 6.46842, Rotation2d.fromDegrees(-90.0))

		val KL_CLOSE = Pose2d(3.70934, 5.37692, Rotation2d.fromDegrees(-60.0))
		val KL_FAR = Pose2d(3.26809, 6.14118, Rotation2d.fromDegrees(-60.0))

		val KL_AB_CLOSE = Pose2d(3.13834, 4.80592, Rotation2d.fromDegrees(-30.0))
		val KL_AB_FAR = Pose2d(2.37407, 5.24717, Rotation2d.fromDegrees(-30.0))

		val KL_CORAL_STATION = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-125.989))
		val CD_CORAL_STATION = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(125.989))

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
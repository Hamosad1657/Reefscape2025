package frc.robot

import edu.wpi.first.math.geometry.Pose2d

object FieldConstants {
	object Poses {
		val AB_CLOSE = Pose2d()
		val AB_FAR = Pose2d()

		val AB_CD_CLOSE = Pose2d()
		val AB_CD_FAR = Pose2d()

		val CD_CLOSE = Pose2d()
		val CD_FAR = Pose2d()

		val CD_EF_CLOSE = Pose2d()
		val CD_EF_FAR = Pose2d()

		val EF_CLOSE = Pose2d()
		val EF_FAR = Pose2d()

		val EF_GH_CLOSE = Pose2d()
		val EF_GH_FAR = Pose2d()

		val GH_CLOSE = Pose2d()
		val GH_FAR = Pose2d()

		val GH_IJ_CLOSE = Pose2d()
		val GH_IJ_FAR = Pose2d()

		val IJ_CLOSE = Pose2d()
		val IJ_FAR = Pose2d()

		val IJ_KL_CLOSE = Pose2d()
		val IJ_KL_FAR = Pose2d()

		val KL_CLOSE = Pose2d()
		val KL_FAR = Pose2d()

		val KL_AB_CLOSE = Pose2d()
		val KL_AB_FAR = Pose2d()

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
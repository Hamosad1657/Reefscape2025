package frc.robot

import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d

object FieldConstants {
	object Poses {
		val AB_CLOSE = Pose2d(2.98534, 4.02592, Rotation2d())
		val AB_FAR = Pose2d(1.96434, 4.02592, Rotation2d())

		val AB_CD_CLOSE = Pose2d(3.18683, 3.27392, Rotation2d())
		val AB_CD_FAR = Pose2d(2.30262, 2.76342, Rotation2d())

		val CD_CLOSE = Pose2d(3.73734, 2.72342, Rotation2d())
		val CD_FAR = Pose2d(3.22684, 1.83920, Rotation2d())

		val CD_EF_CLOSE = Pose2d(4.48934, 2.52192, Rotation2d())
		val CD_EF_FAR = Pose2d(4.48934, 1.50092, Rotation2d())

		val EF_CLOSE = Pose2d(5.24134, 2.72342, Rotation2d())
		val EF_FAR = Pose2d(5.75184, 1.83920, Rotation2d())

		val EF_GH_CLOSE = Pose2d(5.79184, 3.27392, Rotation2d())
		val EF_GH_FAR = Pose2d(6.67605, 2.76342, Rotation2d())

		val GH_CLOSE = Pose2d(5.99334, 4.02592, Rotation2d())
		val GH_FAR = Pose2d(7.01434, 4.02592, Rotation2d())

		val GH_IJ_CLOSE = Pose2d(5.79184, 4.77792, Rotation2d())
		val GH_IJ_FAR = Pose2d(6.67605, 5.28842, Rotation2d())

		val IJ_CLOSE = Pose2d(5.24134, 5.32842, Rotation2d())
		val IJ_FAR = Pose2d(5.75184, 6.21263, Rotation2d())

		val IJ_KL_CLOSE = Pose2d(4.48934, 5.52992, Rotation2d())
		val IJ_KL_FAR = Pose2d(4.48934, 6.55092, Rotation2d())

		val KL_CLOSE = Pose2d(3.73734, 5.32842, Rotation2d())
		val KL_FAR = Pose2d(3.22684, 6.21263, Rotation2d())

		val KL_AB_CLOSE = Pose2d(3.18683, 4.77792, Rotation2d())
		val KL_AB_FAR = Pose2d(2.30262, 5.28842, Rotation2d())

		val KL_CORAL_STATION = Pose2d(0.0, 0.0, Rotation2d(-125.989))
		val CD_CORAL_STATION = Pose2d(0.0, 0.0, Rotation2d(125.989))

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
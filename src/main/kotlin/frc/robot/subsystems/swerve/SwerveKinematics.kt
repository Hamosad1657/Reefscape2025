package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

fun getAngleBetweenTranslations(translation1: Translation2d, translation2: Translation2d): Rotation2d {
	return (translation2 - translation1).angle
}

fun Pose2d.displaceByRelativeTranslation(relativeTranslation: Translation2d): Pose2d =
	Pose2d(
		this.translation + relativeTranslation.rotateBy(this.rotation),
		this.rotation,
	)
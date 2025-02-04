package com.hamosad1657.lib.vision.apriltags

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3

class RobotPoseStdDevs(
	translationX: Double,
	translationY: Double,
	rotation: Double,
) : Matrix<N3, N1>(Nat.N3(), Nat.N1()) {
	init {
		this[0, 0] = translationX
		this[1, 0] = translationY
		this[2, 0] = rotation
	}
}
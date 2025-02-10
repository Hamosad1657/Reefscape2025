package frc.robot.field

import frc.robot.field.AlgaeHeight.HIGH
import frc.robot.field.AlgaeHeight.LOW
import edu.wpi.first.wpilibj.DriverStation

/** Represents a pipe on a reef. */
class Pipe private constructor(val letter: Char) {
	val side: ReefSide
		get() = when (letter) {
		'A', 'B' -> ReefSide.AB
		'C', 'D' -> ReefSide.CD
		'E', 'F' -> ReefSide.EF
		'G', 'H' -> ReefSide.GH
		'I', 'J' -> ReefSide.IJ
		'K', 'L' -> ReefSide.KL
		else -> ReefSide.AB.also {
			DriverStation.reportError("Impossible pipe declared, unable to access reef side.", true)
		}
	}

	companion object {
		val A = Pipe('A')
		val B = Pipe('B')
		val C = Pipe('C')
		val D = Pipe('D')
		val E = Pipe('E')
		val F = Pipe('F')
		val G = Pipe('G')
		val H = Pipe('H')
		val I = Pipe('I')
		val J = Pipe('J')
		val K = Pipe('K')
		val L = Pipe('L')
	}
}

/** Represents one of the levels of a pipe (1->4). Use the companion object. */
class PipeLevel private constructor(val number: Int) {
	companion object {
		val L1 = PipeLevel(1)
		val L2 = PipeLevel(2)
		val L3 = PipeLevel(3)
		val L4 = PipeLevel(4)
	}

	operator fun compareTo(other: PipeLevel): Int = this.number - other.number
}

/** Represents one branch on a reef. */
data class Branch(val pipe: Pipe, val level: PipeLevel)

enum class AlgaeHeight {
	LOW,
	HIGH,
}

/** Represents one Algae on a reef. **/
class ReefAlgae private constructor(val side: ReefSide, val height: AlgaeHeight) {
	companion object {
		val AB = ReefAlgae(ReefSide.AB, HIGH)
		val CD = ReefAlgae(ReefSide.CD, LOW)
		val EF = ReefAlgae(ReefSide.EF, HIGH)
		val GH = ReefAlgae(ReefSide.GH, LOW)
		val IJ = ReefAlgae(ReefSide.IJ, HIGH)
		val KL = ReefAlgae(ReefSide.KL, LOW)
	}
}

/** Represents a side on the reef. */
class ReefSide private constructor(
	/** The number of this side, going from 0 -> 5. */
	val number: Int
) {
	init {
		if (number !in 0..5) DriverStation.reportError("Invalid reef side", true)
	}
	companion object {
		val AB = ReefSide(0)
		val CD = ReefSide(1)
		val EF = ReefSide(2)
		val GH = ReefSide(3)
		val IJ = ReefSide(4)
		val KL = ReefSide(5)
	}
	val sideName get() = when (number) {
		0 -> "AB"
		1 -> "CD"
		2 -> "EF"
		3 -> "GH"
		4 -> "IJ"
		5 -> "KL"
		else -> ""
	}
}

enum class CoralStation {
	KL,
	CD,
}
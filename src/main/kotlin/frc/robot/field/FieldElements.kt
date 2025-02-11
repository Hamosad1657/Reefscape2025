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
enum class PipeLevel(val number: Int) {
	L1(1),
	L2(2),
	L3(3),
	L4(4),
}

/** Represents one branch on a reef. */
data class Branch(val pipe: Pipe, val level: PipeLevel)

enum class AlgaeHeight {
	LOW,
	HIGH,
}

/** Represents one Algae on a reef. **/
enum class ReefAlgae(val side: ReefSide, val height: AlgaeHeight) {
	AB(ReefSide.AB, HIGH),
	CD(ReefSide.CD, LOW),
	EF(ReefSide.EF, HIGH),
	GH(ReefSide.GH, LOW),
	IJ(ReefSide.IJ, HIGH),
	KL(ReefSide.KL, LOW),
}

/** Represents a side on the reef. */
enum class ReefSide(val number: Int) {
	AB(0),
	CD(1),
	EF(2),
	GH(3),
	IJ(4),
	KL(5),
}

enum class CoralStation {
	KL,
	CD,
}
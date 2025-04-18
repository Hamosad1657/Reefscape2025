package frc.robot.field

import frc.robot.field.AlgaeHeight.HIGH
import frc.robot.field.AlgaeHeight.LOW
import edu.wpi.first.wpilibj.DriverStation

/** Represents a pipe on a reef. */
enum class Pipe(val letter: Char) {
	A('A'),
	B('B'),
	C('C'),
	D('D'),
	E('E'),
	F('F'),
	G('G'),
	H('H'),
	I('I'),
	J('J'),
	K('K'),
	L('L');

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
	KL(5);

	val left = when (number) {
		0 -> Pipe.A
		1 -> Pipe.C
		2 -> Pipe.E
		3 -> Pipe.G
		4 -> Pipe.I
		5 -> Pipe.K
		else -> Pipe.A
	}
	val right = when (number) {
		0 -> Pipe.B
		1 -> Pipe.D
		2 -> Pipe.F
		3 -> Pipe.H
		4 -> Pipe.J
		5 -> Pipe.L
		else -> Pipe.B
	}
}

enum class CoralStation {
	KL,
	CD,
}
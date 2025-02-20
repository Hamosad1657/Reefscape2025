package frc.robot.vision

import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import kotlin.math.absoluteValue


//object CoralVision {
//	private var table: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight")
//	private val tx: NetworkTableEntry = table.getEntry("tx")
//
//	private val angleTolerance: Rotation2d = 5.0.degrees
//	val isWithinTolerance get() = coralAngleToCenter.degrees.absoluteValue < angleTolerance.degrees
//	val coralAngleToCenter: Rotation2d get() = Rotation2d.fromDegrees(tx.getDouble(0.0))
//}
package frc.robot.autonomous.segments

import com.hamosad1657.lib.CoralStation
import com.hamosad1657.lib.ReefSide
import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.FieldConstants
import frc.robot.commands.*
import frc.robot.commands.GrabberEjectMode.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

class EjectAlgaeInNet(
	startingSide: ReefSide,
	endingSide: ReefSide,
	isClockwise: Boolean,
	private val netPose: Int,
): AutonomousSegment(startingSide, startingSide, isClockwise) {
	override fun generateCommand(alliance: Alliance) = withName("eject algae in net ") {
		// Get to Net Position
		SwerveSubsystem.followPathCommand(
			PathPlannerPath.fromPathFile("${startingSide.sideName}-far to netPosition"),
			alliance == Alliance.Red,
		)
		//Tell elevator to be in state
		ElevatorJointSubsystem.maintainElevatorJointStateCommand(
			ElevatorJointState.NET,
			true
		)
		// Wait until elevator state is in tolerance
		waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
			//Eject in processor
			(GrabberSubsystem.ejectCommand(NET) withTimeout(2.0) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED }) andThen
			// Get to the ending pose side
			SwerveSubsystem.alignToPoseCommand(
				{
					val pose = FieldConstants.Poses.FAR_POSES[endingSide.number * 2]
					if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
				},
				true,
			)andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
	}
}
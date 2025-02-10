package frc.robot.autonomous.segments

import com.hamosad1657.lib.ReefSide
import com.hamosad1657.lib.commands.*
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

class EjectAlgaeInProcessorSegment(
	private val startingReefSide: ReefSide,
	isClockwise: Boolean,
	): AutonomousSegment(startingReefSide, startingReefSide, isClockwise) {
		override fun generateCommand(alliance: Alliance) = withName("eject algae in processor") {
			// Get to Processor
			SwerveSubsystem.alignToPoseCommand(
				{
					val pose = FieldConstants.Poses.AT_PROCESSOR
					if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
				},
				true,
			)
			//Tell elevator to be in state
			ElevatorJointSubsystem.maintainElevatorJointStateCommand(
				ElevatorJointState.PROCESSOR,
				true
			)
			// Wait until elevator state is in tolerance
			waitUntil { ElevatorJointSubsystem.isWithinTolerance } andThen
			//Eject in processor
				(GrabberSubsystem.ejectCommand(PROCESSOR) withTimeout(2.0) finallyDo { LEDsSubsystem.currentMode = ACTION_FINISHED }) andThen
			// Get back to the pose around the reef
			SwerveSubsystem.alignToPoseCommand(
				{
					val pose = FieldConstants.Poses.FAR_POSES[startingReefSide.number * 2]
					if (alliance == Red) FieldConstants.Poses.mirrorPose(pose) else pose
				},
				true,
				)andThen waitUntil { ElevatorJointSubsystem.isWithinTolerance }
		}
	}
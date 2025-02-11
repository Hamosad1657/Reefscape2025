package frc.robot

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.controllers.HaCommandPS4Controller
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    private const val JOYSTICK_DEADBAND = 0.06
    private const val EJECT_TIMEOUT: Seconds = 2.0

    var elevatorJointState = JointedElevatorState.INTAKE
    var grabberEjectMode = GrabberEjectMode.L1

    var shouldAlignToRightPipe = false


    private val controllerA = HaCommandPS4Controller(JOYSTICK_DEADBAND, RobotMap.DRIVER_A_CONTROLLER_PORT)

    init
    {
        configureDefaultCommands()
        configureBindings()
    }

    private fun configureDefaultCommands() {
        SwerveSubsystem.defaultCommand = SwerveSubsystem.angularVelocityDriveCommand(
            { controllerA.leftY },
            { controllerA.leftX },
            { controllerA.rightX },
            true,
        )

        JointedElevatorSubsystem.defaultCommand = JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false, JointedElevatorState.INTAKE)

        GrabberSubsystem.defaultCommand = GrabberSubsystem.runOnce { GrabberSubsystem.stopMotor() }

        IntakeSubsystem.defaultCommand = IntakeSubsystem.retractIntakeCommand()
    }

    private fun configureBindings()
    {
        with(controllerA) {
            options().onTrue(SwerveSubsystem.runOnce { SwerveSubsystem.zeroGyro() })
            share().onTrue(SwerveSubsystem.runOnce { SwerveSubsystem.setGyro(180.degrees) })

            R2().whileTrue(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true) { elevatorJointState })
            R1().whileTrue(GrabberSubsystem.ejectCommand({ grabberEjectMode }, false) withTimeout(EJECT_TIMEOUT))

            L1().toggleOnTrue(
                intakeCoralFromGroundCommand() // TODO: raceWith alignToCoralDriveCommand()
            )

            triangle().toggleOnTrue(SwerveSubsystem.alignToPipeCommand(
                { SwerveSubsystem.closestReefSide.let { if (shouldAlignToRightPipe) it.right else it.left } }, Robot.alliance)
            )
            square().toggleOnTrue(intakeCoralFromCoralStationCommand())

            // TODO: Pathfinding
        }
    }

    fun getAutonomousCommand(): Command?
    {
        // TODO: Implement properly
        return null
    }
}
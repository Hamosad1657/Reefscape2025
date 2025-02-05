package frc.robot

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.controllers.HaCommandPS4Controller
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.*
import frc.robot.subsystems.elevator.joint.ElevatorJointSubsystem
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem
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
    private const val JOYSTICK_DEADBAND = 0.05
    private const val EJECT_TIMEOUT: Seconds = 2.0

    var elevatorJointState = ElevatorJointState.INTAKE
    var grabberEjectMode = GrabberEjectMode.L1


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

        ElevatorJointSubsystem.defaultCommand = ElevatorJointSubsystem.maintainElevatorJointStateCommand(ElevatorJointState.INTAKE, false)

        GrabberSubsystem.defaultCommand = GrabberSubsystem.runOnce { GrabberSubsystem.stopMotor() }

        IntakeSubsystem.defaultCommand = IntakeSubsystem.retractIntakeCommand()
    }

    private fun configureBindings()
    {
        with(controllerA) {
            options().onTrue(SwerveSubsystem.runOnce { SwerveSubsystem.zeroGyro() })
            share().onTrue(SwerveSubsystem.runOnce { SwerveSubsystem.setGyro(180.degrees) })

            R2().whileTrue(ElevatorJointSubsystem.maintainElevatorJointStateCommand({ elevatorJointState }, useLEDs = true))
            R1().whileTrue(GrabberSubsystem.ejectCommand{ grabberEjectMode } withTimeout(EJECT_TIMEOUT))

            L1().toggleOnTrue(
                intakeCoralFromGroundCommand() // TODO: raceWith alignToCoralCommand()
            )

            // TODO: Triangle().toggleOnTrue(alignToClosestPipeCommand())
            square().toggleOnTrue(intakeCoralFromCoralStationCommand())
            // TODO: Circle().whileTrue(alignToCoralStationCommand())

            // TODO: Pathfinding
        }
    }

    fun getAutonomousCommand(): Command?
    {
        // TODO: Implement properly
        return null
    }
}
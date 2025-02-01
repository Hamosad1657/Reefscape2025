package frc.robot

import com.hamosad1657.lib.controllers.HaCommandPS4Controller
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.*
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsSubsystem

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
    private const val JOYSTICK_DEADBAND = 0.02

    private val controllerA = HaCommandPS4Controller(JOYSTICK_DEADBAND, RobotMap.DRIVER_A_CONTROLLER_PORT)

    init
    {
        configureBindings()
    }

    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings()
    {
        controllerA.square().onTrue(intakeCoralFromGroundCommand())
    }

    fun getAutonomousCommand(): Command?
    {
        // TODO: Implement properly
        return null
    }
}
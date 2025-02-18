package frc.robot

import com.hamosad1657.lib.controllers.HaCommandPS4Controller
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rotations
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.*
import frc.robot.commands.JointedElevatorState.*
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem

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
    private const val DEADBAND = 0.1
    private val testingController = HaCommandPS4Controller(DEADBAND, RobotMap.TESTING_CONTROLLER_PORT)

    init
    {
        configureDefaultCommands()
        sendSubsystemInfo()
        configureBindings()
    }

    private fun sendSubsystemInfo() {
        SmartDashboard.putData(JointedElevatorSubsystem)
    }

    private fun configureDefaultCommands() {

    }

    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings()
    {
        testingController.square().whileTrue(JointedElevatorSubsystem.test_elevatorMotorsSetVoltageCommand { testingController.leftY * 12.0 })
        testingController.triangle().whileTrue(JointedElevatorSubsystem.test_angleMotorSetVoltageCommand { testingController.leftY * 2.0 })

        testingController.circle().whileTrue(JointedElevatorSubsystem.maintainElevatorRotationCommand { 1.0.rotations })
        testingController.cross().whileTrue(JointedElevatorSubsystem.maintainElevatorRotationCommand { 2.0.rotations })

        testingController.R1().whileTrue(JointedElevatorSubsystem.maintainGrabberAngleCommand { 30.0.degrees })
        testingController.L1().whileTrue(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(JointedElevatorState.L3))

        testingController.R2().whileTrue(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(JointedElevatorState.L2))
        testingController.L2().whileTrue(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(JointedElevatorState.L4))
    }

    fun getAutonomousCommand(): Command?
    {
        // TODO: Implement properly
        return null
    }
}
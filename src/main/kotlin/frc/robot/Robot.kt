package frc.robot

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType.kError
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * The functions in this object (which basically functions as a singleton class) are called automatically
 * corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot()
{
    val alliance: Alliance
        get() {
        val allianceOptional = DriverStation.getAlliance()
        return if (allianceOptional.isPresent) {
            allianceOptional.get()
        } else Alliance.Blue.also {
            Alert("Robot alliance could not be determined", kError).set(true)
            DriverStation.reportError("Robot alliance could not be determined", true)
        }
    }

    const val isTesting = true
    const val isCompetition = false

    private var autonomousCommand: Command? = null

    init
    {
        // Kotlin initializer block, which effectually serves as the constructor code.
        // https://kotlinlang.org/docs/classes.html#constructors
        // This work can also be done in the inherited `robotInit()` method. But as of the 2025 season the 
        // `robotInit` method's Javadoc encourages using the constructor and the official templates
        // moved initialization code out `robotInit` and into the constructor. We follow suit in Kotlin.
        
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics.
        // Please retain this line so that Kotlin's growing use by teams is seen by FRC/WPI.
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer
    }


    override fun robotPeriodic()
    {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit()
    {

    }

    override fun disabledPeriodic()
    {
        if (autonomousCommand == null) autonomousCommand = RobotContainer.getAutonomousCommand()
    }

    override fun autonomousInit()
    {
        autonomousCommand?.schedule()
    }

    override fun autonomousPeriodic()
    {

    }

    override fun teleopInit()
    {
        autonomousCommand?.cancel()
    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic()
    {

    }

    override fun testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic()
    {

    }

    override fun simulationInit()
    {

    }

    override fun simulationPeriodic()
    {

    }
}
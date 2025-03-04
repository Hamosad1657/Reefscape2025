package frc.robot

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.controllers.HaCommandPS4Controller
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.ScoringMode.*
import frc.robot.autonomous.AutonomousRoutine
import frc.robot.autonomous.goForwardRoutine
import frc.robot.autonomous.scoreL1IntakePosition1Routine
import frc.robot.autonomous.scoreL1IntakePosition2Routine
import frc.robot.autonomous.scoreL1IntakePosition3Routine
import frc.robot.autonomous.scoreL1IntakePosition4Routine
import frc.robot.autonomous.scoreL1IntakePosition5Routine
import frc.robot.autonomous.scoreL1IntakePosition6Routine
import frc.robot.commands.*
import frc.robot.commands.GrabberVoltageMode.*
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.jointedElevator.JointedElevatorSubsystem
import frc.robot.subsystems.leds.LEDsSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import java.io.File

enum class ScoringMode(val elevatorJointState: JointedElevatorState, val grabberVoltageMode: GrabberVoltageMode) {
    L1(JointedElevatorState.L1, EJECT_TO_L1),
    L2(JointedElevatorState.L2, EJECT_TO_L2_AND_L3),
    L3(JointedElevatorState.L3, EJECT_TO_L2_AND_L3),
    L4(JointedElevatorState.L4, EJECT_TO_L4),
    PROCESSOR(JointedElevatorState.PROCESSOR, EJECT_TO_PROCESSOR),
    NET(JointedElevatorState.NET, EJECT_TO_NET),
    LOW_REEF_ALGAE(JointedElevatorState.LOW_REEF_ALGAE, INTAKE_ALGAE),
    HIGH_REEF_ALGAE(JointedElevatorState.HIGH_REEF_ALGAE, INTAKE_ALGAE),
}

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
    private const val JOYSTICK_DEADBAND = 0.04
    private const val EJECT_TIMEOUT: Seconds = 2.0
    private const val SWERVE_POWER_PROFILE = 3
    private const val CLIMB_POWER_PROFILE = 2

    private const val FREE_DRIVE_TRANSLATION_MULTIPLIER = 0.7
    private const val FREE_DRIVE_ROTATION_MULTIPLIER = 0.5

    private const val EXTENDED_DRIVE_TRANSLATION_MULTIPLIER = 0.3

    private const val INTAKE_ALIGNMENT_TRANSLATION_MULTIPLIER = 0.5
    private const val INTAKE_ALIGNMENT_ROTATION_MULTIPLIER = 0.3

    private const val GO_FORWARD_ROUTINE_NAME = "Go forward"

    private const val POSITION_1_L1_INTAKE_ROUTINE_NAME = "Position 1 intake"
    private const val POSITION_2_L1_INTAKE_ROUTINE_NAME = "Position 2 intake"
    private const val POSITION_3_L1_INTAKE_ROUTINE_NAME = "Position 3 intake"
    private const val POSITION_4_L1_INTAKE_ROUTINE_NAME = "Position 4 intake"
    private const val POSITION_5_L1_INTAKE_ROUTINE_NAME = "Position 5 intake"
    private const val POSITION_6_L1_INTAKE_ROUTINE_NAME = "Position 6 intake"

    private var generatedAutoCommand: Command? = null

    private val autonomousChooser = SendableChooser<String>().apply {
        setDefaultOption("Go forward", GO_FORWARD_ROUTINE_NAME)

        addOption("Position 1 L1 intake", POSITION_1_L1_INTAKE_ROUTINE_NAME)
        addOption("Position 2 L1 intake", POSITION_2_L1_INTAKE_ROUTINE_NAME)
        addOption("Position 3 L1 intake", POSITION_3_L1_INTAKE_ROUTINE_NAME)
        addOption("Position 4 L1 intake", POSITION_4_L1_INTAKE_ROUTINE_NAME)
        addOption("Position 5 L1 intake", POSITION_5_L1_INTAKE_ROUTINE_NAME)
        addOption("Position 6 L1 intake", POSITION_6_L1_INTAKE_ROUTINE_NAME)

        for (fileName in File(Filesystem.getDeployDirectory(), "autonomous").list()?: emptyArray()) {
            addOption("Modular autonomous ${fileName.substringBeforeLast('.')}", fileName.substringBeforeLast('.'))
        }

        onChange { name: String ->
            when(name) {
                GO_FORWARD_ROUTINE_NAME -> goForwardRoutine()

                POSITION_1_L1_INTAKE_ROUTINE_NAME -> generatedAutoCommand = scoreL1IntakePosition1Routine()
                POSITION_2_L1_INTAKE_ROUTINE_NAME -> generatedAutoCommand = scoreL1IntakePosition2Routine()
                POSITION_3_L1_INTAKE_ROUTINE_NAME -> generatedAutoCommand = scoreL1IntakePosition3Routine()
                POSITION_4_L1_INTAKE_ROUTINE_NAME -> generatedAutoCommand = scoreL1IntakePosition4Routine()
                POSITION_5_L1_INTAKE_ROUTINE_NAME -> generatedAutoCommand = scoreL1IntakePosition5Routine()
                POSITION_6_L1_INTAKE_ROUTINE_NAME -> generatedAutoCommand = scoreL1IntakePosition6Routine()

                else -> {
                    generatedAutoCommand = AutonomousRoutine.createFromJSON(name).generateCommand()
                }
            }
        }
    }

    private val translationMultiplierSlewRateLimiter = SlewRateLimiter(0.5, -0.5, FREE_DRIVE_TRANSLATION_MULTIPLIER)

    private val rotationMultiplierSlewRateLimiter = SlewRateLimiter(0.5, -0.5, FREE_DRIVE_ROTATION_MULTIPLIER)

    var currentScoringMode = L1

    var shouldAlignToRightPipe = false

    var shouldIgnoreBeamBreak = false

    private val controllerA = HaCommandPS4Controller(RobotMap.DRIVER_A_CONTROLLER_PORT, JOYSTICK_DEADBAND, SWERVE_POWER_PROFILE)
    private val controllerB = HaCommandPS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT, JOYSTICK_DEADBAND, CLIMB_POWER_PROFILE)

    init
    {
        sendSubsystemInfo()
        configureDefaultCommands()
        configureBindings()
    }

    private fun sendSubsystemInfo() {
        SmartDashboard.putData(autonomousChooser)

        SmartDashboard.putData(GrabberSubsystem)
        SmartDashboard.putData(IntakeSubsystem)
        SmartDashboard.putData(JointedElevatorSubsystem)
        SmartDashboard.putData(SwerveSubsystem)
        SmartDashboard.putData(LEDsSubsystem)
    }

    private fun configureDefaultCommands() {
        SwerveSubsystem.defaultCommand = SwerveSubsystem.angularVelocityDriveCommand(
            { controllerA.leftY },
            { controllerA.leftX },
            { controllerA.rightX },
            isFieldRelative =  true,
            isClosedLoop =  { true },
            { translationMultiplierSlewRateLimiter.calculate(FREE_DRIVE_TRANSLATION_MULTIPLIER) },
            { rotationMultiplierSlewRateLimiter.calculate(FREE_DRIVE_ROTATION_MULTIPLIER) },
        )

        JointedElevatorSubsystem.defaultCommand = JointedElevatorSubsystem.maintainJointedElevatorStateCommand(false, JointedElevatorState.RESTING)

        GrabberSubsystem.defaultCommand = GrabberSubsystem.holdInPlaceCommand()

        IntakeSubsystem.defaultCommand = IntakeSubsystem.maintainAngleCommand { IntakeConstants.FEEDING_ANGLE }
    }

    private fun configureBindings()
    {
        with(controllerA) {
            options().onTrue(SwerveSubsystem.runOnce { SwerveSubsystem.zeroGyro() })
            share().onTrue(SwerveSubsystem.runOnce { SwerveSubsystem.setGyro(180.degrees) })

            R1().whileTrue(GrabberSubsystem.setVoltageCommand(true, false) { currentScoringMode.grabberVoltageMode } withTimeout(EJECT_TIMEOUT))
            R2().whileTrue(JointedElevatorSubsystem.maintainJointedElevatorStateCommand(true) { currentScoringMode.elevatorJointState })

            L1().whileTrue(GrabberSubsystem.setVoltageCommand(true, true) { currentScoringMode.grabberVoltageMode })
            L2().toggleOnTrue(
                IntakeSubsystem.intakeCommand(true) { shouldIgnoreBeamBreak }
            )
//            L2().whileTrue(
//                SwerveSubsystem.rotateToCoralCommand(
//                    { controllerA.leftY },
//                    { controllerA.leftX },
//                    { controllerA.rightX },
//                    isFieldRelative =  true,
//                    isClosedLoop = { true },
//                    { translationMultiplierSlewRateLimiter.calculate(INTAKE_ALIGNMENT_TRANSLATION_MULTIPLIER) },
//                    { rotationMultiplierSlewRateLimiter.calculate(INTAKE_ALIGNMENT_ROTATION_MULTIPLIER) },
//                )
//            )

            triangle().whileTrue(SwerveSubsystem.alignToPipeCommand(
                { SwerveSubsystem.closestReefSide.let { if (shouldAlignToRightPipe) it.right else it.left } }, Robot.alliance)
            )
            square().toggleOnTrue(intakeCoralFromCoralStationCommand())
            circle().toggleOnTrue(IntakeSubsystem.ejectToL1Command(true))

            PS().whileTrue(IntakeSubsystem.ejectFromIntake())
        }

        with(controllerB) {
            triangle().onTrue {
                currentScoringMode = L1
            }

            square().onTrue {
                currentScoringMode = L2
            }

            cross().onTrue {
                currentScoringMode = L3
            }

            circle().onTrue {
                currentScoringMode = L4
            }

            povUp().onTrue {
                currentScoringMode = HIGH_REEF_ALGAE
            }

            povDown().onTrue {
                currentScoringMode = LOW_REEF_ALGAE
            }

            povRight().onTrue {
                currentScoringMode = PROCESSOR
            }

            povLeft().onTrue {
                currentScoringMode = NET
            }

            R1().onTrue {
                shouldAlignToRightPipe = true
            }

            L1().onTrue {
                shouldAlignToRightPipe = false
            }

            L2().toggleOnTrue(
                loadCoralFromIntake()
            )

            PS().onTrue {
                shouldIgnoreBeamBreak = !shouldIgnoreBeamBreak
            }
        }
    }

    fun getAutonomousCommand(): Command
    {
        return generatedAutoCommand?: goForwardRoutine()
    }
}
package frc.robot.subsystems.leds

import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode
import frc.robot.subsystems.leds.LEDsConstants as Constants
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*

object LEDsSubsystem: SubsystemBase("LEDs") {
	var currentMode: LEDsMode = DEFAULT

	private val ledBuffer = AddressableLEDBuffer(Constants.LENGTH)
	private val ledStrip = AddressableLED(RobotMap.LEDs.PWM_PORT).apply {
		setLength(ledBuffer.length)
		setData(ledBuffer)
		start()
	}

	private val timer = Timer()

	private fun applyFlash(color: Color, blinkTime: Seconds) {
		timer.start()
		LEDPattern.solid(color).blink(Units.Seconds.of(blinkTime)).applyTo(ledBuffer)
	}

	private fun handleFlash() {
		if (timer.hasElapsed(Constants.FLASH_TIMEOUT)) {
			currentMode = DEFAULT
			timer.reset()
			timer.stop()
		}
	}

	private fun handleLEDs() {
		when (currentMode) {
			DEFAULT -> {
				when (Robot.alliance) {
					Blue -> {
						currentMode = BLUE_ALLIANCE
						LEDPattern.solid(Color(0,255,0)).applyTo(ledBuffer)
					}
					Red -> {
						currentMode = RED_ALLIANCE
						LEDPattern.solid(Color(0,255,0)).applyTo(ledBuffer)
					}
					else -> LEDPattern.kOff.applyTo(ledBuffer)
				}
			}

			// For some reason Color.kGreen is Blue and the other way around so I swapped them
			LOADING_FROM_CORAL_STATION -> {
				LEDPattern.solid(Color(0,0,255)).applyTo(ledBuffer)
			}
			BLUE_ALLIANCE -> {
				LEDPattern.solid(Color(0,255,0)).applyTo(ledBuffer)
			}
			RED_ALLIANCE -> {
				LEDPattern.solid(Color(0,255,0)).applyTo(ledBuffer)
			}

			ACTION_FINISHED -> {
				applyFlash(Color(0,0,255), Constants.FAST_BLINK_TIME)
				handleFlash()
			}
			EJECTING -> {
				applyFlash(Color(255,0,255), Constants.FAST_BLINK_TIME)
				handleFlash()
			}
			REACHED_SETPOINT -> {
				applyFlash(Color(0,255,0), Constants.FAST_BLINK_TIME)
				handleFlash()
			}
			ACTION_FAILED -> {
				applyFlash(Color.kRed, Constants.FAST_BLINK_TIME)
				handleFlash()
			}

			RAINBOW_STATIC -> {
				LEDPattern.rainbow(255, 128).applyTo(ledBuffer)
			}
			RAINBOW_SCROLL -> {
				LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(
					MetersPerSecond.of(Constants.SCROLL_SPEED),
					Meters.of(Constants.LED_SPACING.asMeters),
				).applyTo(ledBuffer)
			}

			else -> {
				currentMode = DEFAULT
			}
		}
	}

	// --- Periodic ---

	override fun periodic() {
		handleLEDs()
		ledStrip.setData(ledBuffer)
	}
}
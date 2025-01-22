package frc.robot.subsystems.leds

import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants as Constants
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*

object LEDsSubsystem: SubsystemBase("LEDs") {
	var currentMode = DEFAULT
		set(value) {
			when (value) {
				DEFAULT -> {
					LEDPattern.kOff.applyTo(ledBuffer)
				}

				GREEN_STATIC -> {
					LEDPattern.solid(Color.kGreen).applyTo(ledBuffer)
				}
				YELLOW_STATIC -> {
					LEDPattern.solid(Color.kYellow).applyTo(ledBuffer)
				}
				BLUE_STATIC -> {
					LEDPattern.solid(Color.kBlue).applyTo(ledBuffer)
				}
				RED_STATIC -> {
					LEDPattern.solid(Color.kRed).applyTo(ledBuffer)
				}

				GREEN_FLASH -> {
					applyFlash(Color.kGreen, false)
				}
				YELLOW_FLASH -> {
					applyFlash(Color.kYellow, false)
				}
				BLUE_FLASH -> {
					applyFlash(Color.kBlue, false)
				}
				RED_FLASH -> {
					applyFlash(Color.kRed, false)
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
			}
			field = value
		}

	private val ledBuffer = AddressableLEDBuffer(Constants.LENGTH)
	private val ledStrip = AddressableLED(RobotMap.LEDs.PWM_PORT).apply {
		setLength(ledBuffer.length)
		setData(ledBuffer)
		start()
	}

	private val timer = Timer()

	private fun applyFlash(color: Color, slow: Boolean) {
		timer.start()
		LEDPattern.solid(color).blink(Seconds.of(if (slow) Constants.SLOW_BLINK_TIME else Constants.FAST_BLINK_TIME)).applyTo(ledBuffer)
	}

	private fun handleFlash() {
		if (timer.hasElapsed(Constants.FLASH_TIMEOUT)) {
			currentMode = DEFAULT
			timer.reset()
		}
	}

	private fun handleLEDs() {
		when (currentMode) {
			GREEN_FLASH -> {
				handleFlash()
			}
			YELLOW_FLASH -> {
				handleFlash()
			}
			BLUE_FLASH -> {
				handleFlash()
			}
			RED_FLASH -> {
				handleFlash()
			}

			else -> {

			}
		}
	}

	// --- Periodic ---

	override fun periodic() {
		handleLEDs()
		ledStrip.setData(ledBuffer)
	}
}
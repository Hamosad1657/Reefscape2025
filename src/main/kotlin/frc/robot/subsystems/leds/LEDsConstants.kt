package frc.robot.subsystems.leds

import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.Mps
import com.hamosad1657.lib.units.Seconds

object LEDsConstants {
	const val LENGTH = 0

	const val FLASH_TIMEOUT: Seconds = 1.5
	const val FAST_BLINK_TIME: Seconds = 0.1
	const val SLOW_BLINK_TIME: Seconds = 0.4

	val LED_SPACING = Length.fromMeters(1 / 30.0)
	const val SCROLL_SPEED: Mps = 1.0

	enum class LEDsMode {
		DEFAULT,

		GREEN_STATIC,
		LOADING_FROM_CORAL_STATION, //yellow static
		BLUE_STATIC,
		RED_STATIC,

		// All flash modes stay on for [FLASH_TIMEOUT] before going back to default, with a 0.2 second period
		ACTION_FINISHED, //green flash
		PLACING_CORAL, //yellow flash
		REACHED_SETPOINT, //blue flash
		ACTION_FAILED, //red flash

		RAINBOW_STATIC,
		RAINBOW_SCROLL,
	}
}
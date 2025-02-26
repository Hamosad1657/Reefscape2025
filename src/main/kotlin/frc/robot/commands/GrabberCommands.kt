package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rotations
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.commands.GrabberVoltageMode.*
import frc.robot.subsystems.grabber.GrabberConstants
import frc.robot.subsystems.grabber.GrabberSubsystem
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.ACTION_FINISHED
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.EJECTING
import frc.robot.subsystems.leds.LEDsSubsystem

enum class GrabberVoltageMode {
	INTAKE_ALGAE,
	EJECT_TO_L1,
	EJECT_TO_L2_AND_L3,
	EJECT_TO_L4,
	EJECT_TO_PROCESSOR,
	EJECT_TO_NET,
}

fun GrabberSubsystem.setVoltageCommand(useLEDs: Boolean, invert: Boolean, mode: GrabberVoltageMode) = withName("Eject from grabber") {
	setVoltageCommand(useLEDs, invert) { mode }
}

fun GrabberSubsystem.setVoltageCommand(useLEDs: Boolean, invert: Boolean, mode: () -> GrabberVoltageMode) = withName("Eject from grabber") {
	run {
		setMotorVoltage(
			when (mode()) {
				INTAKE_ALGAE -> GrabberConstants.INTAKE_ALGAE_VOLTAGE * (if (invert) -1 else 1)
				EJECT_TO_L1, EJECT_TO_L2_AND_L3, EJECT_TO_L4 -> GrabberConstants.EJECT_CORAL_VOLTAGE * (if (invert) -1 else 1)
				EJECT_TO_PROCESSOR -> GrabberConstants.EJECT_ALGAE_TO_PROCESSOR_VOLTAGE * (if (invert) -1 else 1)
				EJECT_TO_NET -> GrabberConstants.EJECT_ALGAE_TO_NET_VOLTAGE * (if (invert) -1 else 1)
			}
		)
		if (useLEDs){
			LEDsSubsystem.currentMode = EJECTING
		}
	} finallyDo {
		stopMotor()
		if (useLEDs) {
			LEDsSubsystem.currentMode = ACTION_FINISHED
		}
	}
}

fun GrabberSubsystem.holdInPlaceCommand() = withName("Hold algae") {
	run {
		setMotorSetpoint(0.0.rotations)
	}
}

fun GrabberSubsystem.stopMotorCommand() = withName("Stop motor") {
	run { stopMotor() }
}

fun GrabberSubsystem.loadFromIntakeCommand() = withName("Load from intake command") {
	(run {
		setMotorVoltage(GrabberConstants.LOAD_FROM_INTAKE_VOLTAGE)
	} until { isBeamBreakInterfered }) andThen (
		(run {
			setMotorVoltage(GrabberConstants.LOAD_FROM_INTAKE_VOLTAGE)
		} withTimeout(1.0)) andThen (run {
			setMotorVoltage(GrabberConstants.LOAD_FROM_INTAKE_VOLTAGE)
		} until { !isBeamBreakInterfered })) andThen (
			run {
				setMotorVoltage(GrabberConstants.CLOCK_CORAL_VOLTAGE)
			} withTimeout(0.35)
		)
}

enum class LoadFromCoralStationState(val shouldExitState: () -> Boolean) {
	Loading(shouldExitState = {
		GrabberSubsystem.isBeamBreakInterfered
	}),
	Holding(shouldExitState = {
		GrabberSubsystem.isInTolerance
	}),
	Finished(shouldExitState = {
		false
	})
}

fun GrabberSubsystem.loadFromCoralStationCommand() = withName("Load from coral station") {
	var loadFromCoralStationState = LoadFromCoralStationState.Loading
	run {
		when (loadFromCoralStationState) {
			LoadFromCoralStationState.Loading -> {
				setMotorVoltage(GrabberConstants.INTAKE_FROM_CORAL_STATION_VOLTAGE)
				if (loadFromCoralStationState.shouldExitState()) {
					loadFromCoralStationState = LoadFromCoralStationState.Holding
					setMotorSetpoint(Rotation2d.fromRotations(-3.0))
				}
			}
			LoadFromCoralStationState.Holding -> {
				updateMotorPIDControl()
				if (loadFromCoralStationState.shouldExitState()) loadFromCoralStationState = LoadFromCoralStationState.Finished
			}
			LoadFromCoralStationState.Finished -> {}
		}
	} until { loadFromCoralStationState == LoadFromCoralStationState.Finished } finallyDo { stopMotor() }
}

//--- Test commands ---

fun GrabberSubsystem.test_setWheelsVoltageCommand(voltage: () -> Volts) = withName("Set wheels voltage") {
	run { setMotorVoltage(voltage()) }
}
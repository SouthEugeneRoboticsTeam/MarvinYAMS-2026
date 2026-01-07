package org.sert2521.marvin2026.subsystems.flywheel

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.marvin2026.ElectronicIDs
import org.sert2521.marvin2026.FlywheelsConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object FlywheelSubsystem : SubsystemBase() {
    private val motorTop= SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorBottom = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_BOTTOM_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigTop = SmartMotorControllerConfig(this)
        .withClosedLoopController(FlywheelsConstants.P, 0.0, FlywheelsConstants.D)
        .withFeedforward(SimpleMotorFeedforward(FlywheelsConstants.S, FlywheelsConstants.V, FlywheelsConstants.A))
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Top", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)

    private val motorConfigBottom = SmartMotorControllerConfig(this)
        .withClosedLoopController(FlywheelsConstants.P, 0.0, FlywheelsConstants.D)
        .withFeedforward(SimpleMotorFeedforward(FlywheelsConstants.S, FlywheelsConstants.V, FlywheelsConstants.A))
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Top", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)
}
    private val topSMC = SparkWrapper(motorTop, DCMotor.getNEO(1), motorConfigTop)
    private val bottomSMC = SparkWrapper(motorBottom, DCMotor.getNEO(1), motorConfigBottom)

    private var topLastSetpoint = RPM.zero()
    private var bottomLastSetpoint = RPM.zero()
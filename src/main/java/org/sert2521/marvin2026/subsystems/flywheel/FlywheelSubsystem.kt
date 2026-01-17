package org.sert2521.marvin2026.subsystems.flywheel

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.marvin2026.ElectronicIDs
import org.sert2521.marvin2026.FlywheelsConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry
import java.util.function.Supplier

object FlywheelSubsystem : SubsystemBase() {
    private val motorTop = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)
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

    private val topSMC = SparkWrapper(motorTop, DCMotor.getNEO(1), motorConfigTop)
    private val bottomSMC = SparkWrapper(motorBottom, DCMotor.getNEO(1), motorConfigBottom)
    private val telemetry = MechanismTelemetry()

    private var topLastSetpoint = RPM.zero()
    private var bottomLastSetpoint = RPM.zero()


    init {
        defaultCommand = holdCommand(::topLastSetpoint, ::bottomLastSetpoint)

        telemetry.setupTelemetry("SubsystemName", topSMC)
        telemetry.setupTelemetry("SubsystemName", bottomSMC)
    }

    override fun periodic() {
        topSMC.updateTelemetry()
        bottomSMC.updateTelemetry()
    }

    override fun simulationPeriodic() {
        topSMC.simIterate()
        bottomSMC.simIterate()
    }

    private fun setVelocitiesCommand(velocityTop: AngularVelocity, velocityBottom: AngularVelocity): Command {
        return runOnce {
            topSMC.setVelocity(velocityTop)
            bottomSMC.setVelocity(velocityBottom)
            topLastSetpoint = velocityTop
            bottomLastSetpoint = velocityBottom
        }.until {
            MathUtil.isNear(velocityTop.`in`(RPM), topSMC.mechanismVelocity.`in`(RPM), 10.0)
                    && MathUtil.isNear(velocityBottom.`in`(RPM), bottomSMC.mechanismVelocity.`in`(RPM), 10.0)
        }
    }

    private fun holdCommand(velocityTop: Supplier<AngularVelocity>,
                            velocityBottom: Supplier<AngularVelocity>):Command{
        return runOnce{
            topSMC.setVelocity(velocityTop.get())
            bottomSMC.setVelocity(velocityBottom.get())
            topLastSetpoint = velocityTop.get()
            bottomLastSetpoint = velocityBottom.get()
        }.andThen(
            Commands.idle()
        )
    }

    fun rev(): Command {
        return setVelocitiesCommand(FlywheelsConstants.ShootTarget, FlywheelsConstants.ShootTarget)
    }

    fun stop(): Command {
        return setVelocitiesCommand(RPM.zero(), RPM.zero())
    }
}
package org.sert2521.marvin2026.subsystems.flywheel

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
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

    private var topLastSetpoint = RPM.zero()
    private var bottomLastSetpoint = RPM.zero()


    init {
        defaultCommand = holdCommand(::topLastSetpoint, ::bottomLastSetpoint)

        org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topSMC.setupTelemetry(
            NetworkTableInstance.getDefault().getTable("Tuning")
                .getSubTable("Flywheels"),
            NetworkTableInstance.getDefault().getTable("Mechanisms")
                .getSubTable("Flywheels")
        )
        org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomSMC.setupTelemetry(
            NetworkTableInstance.getDefault().getTable("Tuning")
                .getSubTable("Flywheels"),
            NetworkTableInstance.getDefault().getTable("Mechanisms")
                .getSubTable("Flywheels")
        )
    }

    override fun periodic() {
        org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topSMC.updateTelemetry()
        org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomSMC.updateTelemetry()
        // DogLog.log("Top Flywheel Speed", org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topSMC.mechanismVelocity.`in`(RPM))
        // DogLog.log("Bottom Flywheel Speed", org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomSMC.mechanismVelocity.`in`(RPM))
    }

    override fun simulationPeriodic() {
        org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topSMC.simIterate()
        org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomSMC.simIterate()
    }

    private fun setVelocitiesCommand(velocityTop: AngularVelocity, velocityBottom: AngularVelocity): Command {
        return runOnce {
            org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topSMC.setVelocity(velocityTop)
            org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomSMC.setVelocity(velocityBottom)
            org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topLastSetpoint = velocityTop
            org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomLastSetpoint = velocityBottom
        }.until {
            MathUtil.isNear(velocityTop.`in`(RPM), org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.topSMC.mechanismVelocity.`in`(RPM), 10.0)
                    && MathUtil.isNear(velocityBottom.`in`(RPM), org.sert2521.marvin2026.subsystems.flywheel.FlywheelSubsystem.bottomSMC.mechanismVelocity.`in`(RPM), 10.0)
        }
    }

    private fun holdCommand(velocityTop: Supplier<AngularVelocity>,
                            velocityBottom: Supplier<AngularVelocity>):Command{
        return runOnce{
            topSMC.setVelocity(velocityTop.get())
            bottomSMC.setVelocity(velocityBottom.get())
    //            topLastSetpoint = velocityTop.get()
    //            bottomLastSetpoint = velocityBottom.get()
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
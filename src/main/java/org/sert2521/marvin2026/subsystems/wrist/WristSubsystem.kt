package org.sert2521.marvin2026.subsystems.wrist

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.marvin2026.ElectronicIDs
import org.sert2521.marvin2026.WristConstants
import yams.mechanisms.config.ArmConfig
import yams.mechanisms.positional.Arm
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry


object WristSubsystem : SubsystemBase() {
    private val wristMotor = SparkMax(ElectronicIDs.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            WristConstants.P,
            WristConstants.I,
            WristConstants.D
        )
        .withGearing(WristConstants.wristGearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Wrist Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val smc = SparkWrapper(wristMotor, DCMotor.getNEO(1), motorConfig)

    private val wristConfig = ArmConfig(smc)
        .withMOI(WristConstants.moi.`in`(KilogramSquareMeters))
        .withHardLimit(WristConstants.hardMin, WristConstants.hardMax)
        .withTelemetry("Wrist", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLength(WristConstants.length)
        .withStartingPosition(Rotations.of(0.0))

    private val wrist = Arm(wristConfig)

    private val stallDebouncer = Debouncer(0.2, Debouncer.DebounceType.kRising)

    private val telemetry = MechanismTelemetry()

    private var lastSetpoint = WristConstants.stowPosition

    init {
        telemetry.setupTelemetry("SubsystemName", smc)
    }

    override fun periodic() {
        wrist.updateTelemetry()
    }

    override fun simulationPeriodic() {
        wrist.simIterate()
    }

    private fun setAngleCommand(angle: Angle): Command {
        return wrist.setAngle(angle).until {
            MathUtil.isNear(angle.`in`(Rotations), wrist.angle.`in`(Rotations), 0.05)
        }
    }

    fun toStow(): Command {
        return setAngleCommand(WristConstants.stowPosition)
    }

    fun toIntake(): Command {
        return setAngleCommand(WristConstants.intakePosition)
    }

    fun resetWristCommand(): Command {
        return wrist.set(WristConstants.RESET_DUTY_CYCLE)
            .until{
                stallDebouncer.calculate(wrist.motor.statorCurrent > Amps.of(30.0))
            }.andThen(
                runOnce {
                    wrist.motor.setEncoderPosition(WristConstants.hardMax)
                    stallDebouncer.calculate(false)
                }
            ).finallyDo { interrupted ->
                if (!interrupted) {
                    toStow().schedule()
                }
                wrist.motor.startClosedLoopController()
            }
    }

    fun sysId(): Command {
        return wrist.sysId(Volts.of(12.0), Volts.of(0.2).per(Second), Seconds.of(30.0))
    }
}
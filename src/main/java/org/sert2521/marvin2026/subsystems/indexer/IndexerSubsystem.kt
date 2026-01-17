package org.sert2521.marvin2026.subsystems.indexer

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.marvin2026.ElectronicIDs
import org.sert2521.marvin2026.ElectronicIDs.INDEXER_BEAMBREAK_ID
import org.sert2521.marvin2026.IndexerConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object IndexerSubsystem : SubsystemBase() {
    private val indexerMotor = SparkMax(ElectronicIDs.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val indexerBeambreak = DigitalInput(INDEXER_BEAMBREAK_ID)

    private val indexerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(IndexerConstants.indexerGearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Indexer Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)

    private val indexerSMC = SparkWrapper(indexerMotor, DCMotor.getNEO(1), indexerMotorConfig)

    private val telemetry = MechanismTelemetry()

    init {
        telemetry.setupTelemetry("SubsystemName", indexerSMC)
    }

    override fun periodic() {
        indexerSMC.updateTelemetry()
    }

    override fun simulationPeriodic() {
        indexerSMC.simIterate()
    }

    private fun setIndexerMotor(dutyCycle: Double) {
        indexerSMC.dutyCycle = dutyCycle
    }

    fun index(): Command {
        return runOnce {
            setIndexerMotor(IndexerConstants.INDEXING)
        }.until {
            indexerBeambreak.get()
        }
    }

    fun reverse():Command {
        return runOnce {
            setIndexerMotor(IndexerConstants.REVERSE)
        }.andThen(
            Commands.idle()
        )
    }
}
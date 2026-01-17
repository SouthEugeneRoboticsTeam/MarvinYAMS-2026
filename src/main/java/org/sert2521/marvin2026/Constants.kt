package org.sert2521.marvin2026

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.RPM
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object ElectronicIDs {
    const val FLYWHEEL_MOTOR_TOP_ID = 1
    const val FLYWHEEL_MOTOR_BOTTOM_ID = 2
    const val INDEXER_MOTOR_ID = 3
    const val INDEXER_BEAMBREAK_ID = 4
    const val INTAKE_MOTOR_ID = 5
    const val WRIST_MOTOR_ID = 6
}

object FlywheelsConstants {
    const val P = 0.0
    const val D = 0.0

    const val S = 0.0
    const val V = 0.0
    const val A = 0.0

    val gearing = MechanismGearing (
        GearBox.fromReductionStages(
            1.0
        )
    )

    val ShootTarget = RPM.of(0.0)
}

object IndexerConstants {
    val indexerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            6.75
        )
    )

    const val DEFAULT = 0.0

    const val INDEXING = 0.3

    const val REVERSE = -0.2
}

object IntakeConstants {
    val intakeGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0
        )
    )

    const val INTAKE_SPEED = 0.2
    const val REVERSE_SPEED = -0.2
}

object WristConstants {
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val wristGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0
        )
    )

    val moi = KilogramSquareMeters.of(0.0)

    val length = Inches.of(0.0)

    val hardMin = Degrees.of(0.0)
    val hardMax = Degrees.of(0.0)

    val stowPosition = Degrees.of(0.0)
    val intakePosition = Degrees.of(0.0)

    const val RESET_DUTY_CYCLE = 0.1
}
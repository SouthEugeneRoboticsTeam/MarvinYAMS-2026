package org.sert2521.marvin2026

import edu.wpi.first.units.Units.RPM
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object ElectronicIDs {
    const val FLYWHEEL_MOTOR_TOP_ID = 1
    const val FLYWHEEL_MOTOR_BOTTOM_ID = 2
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
package org.sert2521.marvin2026.subsystems.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Distance
import yams.gearing.GearBox
import yams.gearing.MechanismGearing


object SwerveConstants {
    val driveGearing = MechanismGearing(GearBox.fromReductionStages(6.75))
    val angleGearing = MechanismGearing(GearBox.fromReductionStages(150.0 / 7.0))

    val wheelRadius: Distance = Inches.of(2.0)
    const val WHEEL_COF = 1.54

    val moduleNames = arrayOf("Front Left", "Front Right", "Back Left", "Back Right")
    val moduleTranslations = arrayOf(
        Translation2d(Meters.of(0.288925), Meters.of(0.288925)),
        Translation2d(Meters.of(0.288925), -Meters.of(0.288925)),
        Translation2d(-Meters.of(0.288925), Meters.of(0.288925)),
        Translation2d(-Meters.of(0.288925), -Meters.of(0.288925))
    )
    val moduleZeroRotations = arrayOf(
        Rotations.of(0.08056640624999999),//-0.44222405552864075), // FL
        Rotations.of(0.272705078125),//0.15999986231327054), // FR
        Rotations.of(-0.88427734375),// 0.0022221936378628016), // BL
        Rotations.of(0.681884765625)// -0.567780077457428) // BR
    )
    val encoderIDs = arrayOf(1, 2, 3, 4)
    val driveIDs = arrayOf(5, 6, 7, 8)
    val angleIDs = arrayOf(9, 10, 11, 12)

    // TODO: Tune
    const val DRIVE_P = 0.0
    const val DRIVE_D = 0.0
    const val DRIVE_S = 0.0
    const val DRIVE_V = 0.85
    val driveCurrentLimit = Amps.of(40.0)

    // TODO: Tune
    const val ANGLE_P = 45.0
    const val ANGLE_D = 1.0
    val angleCurrentLimit = Amps.of(40.0)

    // TODO: Tune
    const val AUTO_TRANSLATION_P = 4.0 // 4.0
    const val AUTO_TRANSLATION_I = 0.0
    const val AUTO_TRANSLATION_D = 0.4 // 0.4

    // TODO: Tune
    const val AUTO_HEADING_P = 5.5 // 5.5
    const val AUTO_HEADING_I = 0.0
    const val AUTO_HEADING_D = 0.4 // 0.4

    // TODO: Tune
    const val VISION_TRANSLATION_P = 0.0 // 6.0
    const val VISION_TRANSLATION_D = 0.0 // 0.4

    // TODO: Tune
    const val VISION_HEADING_P = 0.0 // 8.0
    const val VISION_HEADING_D = 0.0 // 0.47

    // TODO: Tune
    const val TRANSLATION_OUTPUT_MIN = 0.0

    val maxSpeed = MetersPerSecond.of(4.571)

    const val SYS_ID_FF_RAMP_RATE = 1.0
}

object DriveConfig {
    const val DRIVE_SPEED = 4.5

    const val ROT_SPEED = 4.0

    const val DRIVE_ACCELERATION = 0.0
}
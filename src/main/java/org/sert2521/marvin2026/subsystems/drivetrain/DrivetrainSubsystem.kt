package org.sert2521.marvin2026.subsystems.drivetrain

import com.ctre.phoenix6.configs.MountPoseConfigs
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Rotation
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.ANGLE_D
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.ANGLE_P
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.DRIVE_D
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.DRIVE_P
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.DRIVE_S
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.DRIVE_V
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.angleCurrentLimit
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.angleGearing
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.angleIDs
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.driveCurrentLimit
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.driveGearing
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.driveIDs
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.encoderIDs
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.moduleNames
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.moduleTranslations
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.moduleZeroRotations
import org.sert2521.marvin2026.subsystems.drivetrain.SwerveConstants.wheelRadius
import yams.mechanisms.config.SwerveModuleConfig
import yams.mechanisms.swerve.SwerveModule
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import kotlin.math.PI
import kotlin.math.hypot

object DrivetrainSubsystem : SubsystemBase() {
    private fun createModule(
        driveMotor: SparkMax, angleMotor: SparkMax, absoluteEncoder: CANcoder,
        moduleName: String, location: Translation2d, rotationZero: Angle
    ): SwerveModule {
        val driveConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withWheelDiameter(wheelRadius * 2.0)
            .withFeedforward(SimpleMotorFeedforward(DRIVE_S, DRIVE_V))
            .withClosedLoopController(DRIVE_P, 0.0, DRIVE_D)
            .withGearing(driveGearing)
            .withMotorInverted(true)
            .withStatorCurrentLimit(driveCurrentLimit)
            .withMomentOfInertia(0.0005267514) // Use the formula MR^2/2
            .withTelemetry("Drive Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val angleConfig = SmartMotorControllerConfig(this)
            .withClosedLoopController(ANGLE_P, 0.0, ANGLE_D)
            .withContinuousWrapping(Radians.of(-PI), Radians.of(PI))
            .withGearing(angleGearing)
            .withMotorInverted(true)
            .withStatorCurrentLimit(angleCurrentLimit)
            .withMomentOfInertia(0.0003195625) // Use the formula MR^2/4 + ML^2/12
            .withTelemetry("Angle Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val driveSMC = SparkWrapper(driveMotor, DCMotor.getNEO(1), driveConfig)
        val angleSMC = SparkWrapper(angleMotor, DCMotor.getNEO(1), angleConfig)

        val moduleConfig = SwerveModuleConfig(driveSMC, angleSMC)
            .withAbsoluteEncoderOffset(rotationZero)
            .withAbsoluteEncoder(
                absoluteEncoder.position.asSupplier()
            )
            .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.LOW)
            .withLocation(location)
            .withOptimization(true)
            .withCosineCompensation(false)

        return SwerveModule(moduleConfig)
    }

    private val modules = Array(4) { index ->
        createModule(
            SparkMax(driveIDs[index], SparkLowLevel.MotorType.kBrushless),
            SparkMax(angleIDs[index], SparkLowLevel.MotorType.kBrushless),
            CANcoder(encoderIDs[index]),
            moduleNames[index],
            moduleTranslations[index],
            moduleZeroRotations[index]
        )
    }


    private val gyroConfig = Pigeon2Configuration()
        .withMountPose(MountPoseConfigs().withMountPosePitch(Degrees.of(90.0)))
        .withPigeon2Features(Pigeon2FeaturesConfigs().withEnableCompass(false))
    private val gyro = Pigeon2(13)
    private val gyroYaw = gyro.yaw.asSupplier()

    private val kinematics = SwerveDriveKinematics(*moduleTranslations)
    private var moduleStates = Array(4) { modules[it].state }

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.kZero,
        Array(4) { modules[it].position },
        Pose2d.kZero
    )

    private var simGyroAngle = Rotation.zero()

    private val field = Field2d()

    private val gyroConnected = Debouncer(0.1)

    private val simTimer = Timer()

    init {
        SmartDashboard.putData(field)
        gyro.configurator.apply(gyroConfig)

        defaultCommand = JoystickDrive(true)
    }

    override fun periodic() {
        modules.forEach { it.updateTelemetry() }
        modules.forEach { it.seedAzimuthEncoder() }

        field.robotPose = poseEstimator.estimatedPosition

        val currentGyroConnected = gyroConnected.calculate(gyro.isConnected)
        if (!currentGyroConnected) {
            DogLog.logFault("Gyro Disconnected", Alert.AlertType.kError)
        }

        poseEstimator.update(Rotation2d(getGyroAngle()), getModulePositions())

        moduleStates = getModuleStates()
        DogLog.log("Drivetrain/SwerveModuleStates/Measured", moduleStates)

        val chassisSpeeds = kinematics.toChassisSpeeds(moduleStates)
        DogLog.log("Drivetrain/ChassisSpeeds/Measured", chassisSpeeds)
        DogLog.log(
            "Drivetrain/ChassisSpeeds/Measured Drive Speed",
            hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        )

        DogLog.log("Drivetrain/Rotation", poseEstimator.estimatedPosition.rotation)

        if (DriverStation.isDisabled()) {
            DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", Array(4) { SwerveModuleState() })
            DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", Array(4) { SwerveModuleState() })
        }
    }

    override fun simulationPeriodic() {
        if (!simTimer.isRunning) {
            simTimer.start()
        }

        modules.forEach { it.simIterate() }

        simGyroAngle = simGyroAngle.plus(
            Radians.of(
                kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * simTimer.get()
            )
        )
        simTimer.reset()
    }

    /* Private Functions */
    /**
     * @return The optimized states
     */
    private fun setModuleStates(vararg states: SwerveModuleState): Array<SwerveModuleState> {
        modules.forEachIndexed { index, module ->
            module.setSwerveModuleState(states[index])
        }
        return Array(4) { modules[it].config.getOptimizedState(states[it]) }
    }

    private fun getGyroAngle(): Angle {
        return if (RobotBase.isReal()) {
            gyroYaw.get()
        } else {
            simGyroAngle
        }
    }

    private fun getModuleStates(): Array<SwerveModuleState> {
        return Array(4) { modules[it].state }
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        return Array(4) { modules[it].position }
    }

    /* Public Functions */
    fun getChassisSpeeds(): ChassisSpeeds {
        return kinematics.toChassisSpeeds(getModuleStates())
    }

    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        DogLog.log("Drivetrain/ChassisSpeeds/Setpoints", speeds)
        DogLog.log(
            "Drivetrain/ChassisSpeeds/Setpoint Drive Speed",
            hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        )

        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.maxSpeed)

        DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", setpointStates)

        // Modules are set here
        val optimizedStates = setModuleStates(*setpointStates)
        DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", optimizedStates)
    }

    fun stopDrivePID() {
        modules.forEach {
            it.config.driveMotor.setFeedback(0.0, 0.0, 0.0)
        }
    }

    fun startDrivePID() {
        modules.forEach {
            it.config.driveMotor.setFeedback(DRIVE_P, 0.0, DRIVE_D)
        }
    }

    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    fun setPose(pose: Pose2d) {
        poseEstimator.resetPose(pose)
    }

    fun setRotation(rotation: Rotation2d){
        poseEstimator.resetRotation(rotation)
    }

//    fun getVisionPose(): Optional<Pose2d> {
//        if (!limelight.latestResults.isPresent) {
//            return Optional.empty()
//        }
//        if (!limelight.latestResults.get().valid) {
//            return Optional.empty()
//        }
//
//        // Finding the biggest tag in view of the limelight, chooses that one for vision align
//        var biggestTag = 0
//        val latestResults = limelight.latestResults.get().targets_Fiducials
//        for (i in latestResults.indices) {
//            if (latestResults[i].ta > latestResults[biggestTag].ta) {
//                biggestTag = i
//            }
//        }
//
//        return Optional.of(latestResults[biggestTag].robotPose_TargetSpace2D)
//    }
//
//    fun getVisionPoseToTarget(target: Pose2d): Pose2d {
//        val visionPose = getVisionPose()
//
//        return if (visionPose.isPresent) {
//            target.relativeTo(visionPose.get())
//        } else {
//            Pose2d.kZero
//        }
//    }

    fun runFFCharacterization(output: Double): Double {
        modules.forEach {
            it.config.azimuthMotor.setPosition(Rotations.zero())
            it.config.driveMotor.voltage = Volts.of(output)
        }

        var avgVelocity = 0.0
        modules.forEach {
            avgVelocity += (it.state.speedMetersPerSecond / wheelRadius.`in`(Meters)) / 4.0
        }
        return avgVelocity
    }
}
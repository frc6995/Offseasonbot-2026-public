package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.EstimationMode;
import frc.robot.subsystems.vision.apriltag.AprilTagVision;
import frc.robot.subsystems.vision.apriltag.RealATLimelightVision.LimelightConstants;
import frc.robot.subsystems.vision.photon.RealPhotonATVision;
import frc.robot.util.LimelightHelpers;

public class ATVision extends SubsystemBase {
    public static class ATVisionConstants {
        public static final String NT_TABLE = "Vision";

        public static final double[] kLimelightMT2StdDevCoefficients = {0.085, 0.0};
        public static final double[] kLimelightMT1StdDevCoefficients = {0.5, 0.5};
        public static final int kOptimalTagCount = 2;

        public static final Pose3d kInitialTurretCameraOffset = solveRobotToCamera(0.0);

        public static final double kMaxAmbiguity = 0.3;

        public static final double kMaxChassisOmegaRadPerSec = Math.PI / 2;

        public static final double kMaxTiltRad = Math.toRadians(20);

        public static final double kEstimateHistorySeconds = 1.0;
        public static final double kMaxEstimateAgeSeconds = 0.4;
        public static final double kClockSkewToleranceSeconds = 0.01;

        public static final double kOffsetTransportLatencySeconds = 0.02;
        public static final double kMaxTurretMismatchDeg = 3.0;
        public static final double kMaxTurretVelDegPerSec = 180.0;
        public static final double kVelocitySampleDtSeconds = 0.02;
    }

    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    private final AprilTagVision limelightVision;
    private final RealPhotonATVision photonVision;
    private final Supplier<SwerveDriveState> swerveState;
    private final Supplier<Rotation3d> gyroRotation;
    private final VisionMeasurementConsumer addVisionMeasurement;
    private final Supplier<Double> turretAngleSupplier;

    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final TimeInterpolatableBuffer<Rotation2d> pushedAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final TimeInterpolatableBuffer<Double> chassisOmegaBuffer =
        TimeInterpolatableBuffer.createDoubleBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final TimeInterpolatableBuffer<Double> rollBuffer =
        TimeInterpolatableBuffer.createDoubleBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final TimeInterpolatableBuffer<Double> pitchBuffer =
        TimeInterpolatableBuffer.createDoubleBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final BooleanPublisher headingSeededPublisher;
    private final StructPublisher<Pose3d> seededPosePublisher;
    private final StructPublisher<Pose3d> robotToCameraPublisher;
    private final DoublePublisher acceptedCountPublisher;
    private final DoublePublisher mismatchPublisher;
    private final DoublePublisher estimateAgePublisher;
    private final StringPublisher rejectReasonPublisher;

    private final DoublePublisher camTurretInputDegPublisher;
    private final DoublePublisher camSentRollDegPublisher;
    private final DoublePublisher camSentPitchDegPublisher;
    private final DoublePublisher camSentYawDegPublisher;
    private final DoublePublisher camLensHeadingDegPublisher;
    private final DoublePublisher camRadialHeadingDegPublisher;
    private final StructPublisher<Pose3d> camLimelightAppliedPublisher;
    private final StructPublisher<Pose3d> camFieldPosePublisher;

    private boolean headingSeeded = false;
    private double lastMismatchDeg = 0;
    private double lastAgeSeconds = 0;
    private String lastRejectReason = "";

    public ATVision(
            AprilTagVision limelightVision,
            RealPhotonATVision photonVision,
            Supplier<SwerveDriveState> swerveState,
            Supplier<Rotation3d> gyroRotation,
            VisionMeasurementConsumer addVisionMeasurement,
            Supplier<Double> turretAngleSupplier) {
        this.limelightVision = limelightVision;
        this.photonVision = photonVision;
        this.swerveState = swerveState;
        this.gyroRotation = gyroRotation;
        this.addVisionMeasurement = addVisionMeasurement;
        this.turretAngleSupplier = turretAngleSupplier;

        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable(ATVisionConstants.NT_TABLE);
        headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();
        seededPosePublisher = visionTable.getStructTopic("SeededPose", Pose3d.struct).publish();
        robotToCameraPublisher = visionTable.getStructTopic("TurretRobotToCamera", Pose3d.struct).publish();
        acceptedCountPublisher = visionTable.getDoubleTopic("AcceptedEstimates").publish();
        mismatchPublisher = visionTable.getDoubleTopic("TurretMismatchDeg").publish();
        estimateAgePublisher = visionTable.getDoubleTopic("EstimateAgeSeconds").publish();
        rejectReasonPublisher = visionTable.getStringTopic("LastRejectReason").publish();

        NetworkTable camTable = visionTable.getSubTable("TurretCamDebug");
        camTurretInputDegPublisher = camTable.getDoubleTopic("TurretInputDeg").publish();
        camSentRollDegPublisher = camTable.getDoubleTopic("SentRollDeg").publish();
        camSentPitchDegPublisher = camTable.getDoubleTopic("SentPitchDeg").publish();
        camSentYawDegPublisher = camTable.getDoubleTopic("SentYawDeg").publish();
        camLensHeadingDegPublisher = camTable.getDoubleTopic("LensHeadingDeg").publish();
        camRadialHeadingDegPublisher = camTable.getDoubleTopic("RadialHeadingDeg").publish();
        camLimelightAppliedPublisher = camTable.getStructTopic("LimelightApplied", Pose3d.struct).publish();
        camFieldPosePublisher = camTable.getStructTopic("FieldPose", Pose3d.struct).publish();
    }

    private void publishTurretCameraDebug(double turretInputDeg, Pose3d robotToCamera, Pose2d robotPose) {
        Rotation3d rot = robotToCamera.getRotation();
        camTurretInputDegPublisher.accept(turretInputDeg);
        camSentRollDegPublisher.accept(Math.toDegrees(rot.getX()));
        camSentPitchDegPublisher.accept(Math.toDegrees(rot.getY()));
        camSentYawDegPublisher.accept(Math.toDegrees(rot.getZ()));

        Translation3d lens = new Translation3d(1, 0, 0).rotateBy(rot);
        Translation3d radial = robotToCamera.getTranslation()
            .minus(TurretConstants.turretCenterPose.getTranslation());
        camLensHeadingDegPublisher.accept(Math.toDegrees(Math.atan2(lens.getY(), lens.getX())));
        camRadialHeadingDegPublisher.accept(Math.toDegrees(Math.atan2(radial.getY(), radial.getX())));

        camLimelightAppliedPublisher.accept(LimelightHelpers.getCameraPose3d_RobotSpace(
            LimelightConstants.LL_IDS[LimelightConstants.kTurretCameraIndex]));

        camFieldPosePublisher.accept(new Pose3d(robotPose).transformBy(
            new Transform3d(robotToCamera.getTranslation(), robotToCamera.getRotation())));
    }

    public static Pose3d solveRobotToCamera(double turretAngleDeg) {
        Rotation3d turretRotation = new Rotation3d(0, 0, -Math.toRadians(turretAngleDeg));

        Pose3d rotatedTurret = new Pose3d(
            TurretConstants.turretCenterPose.getTranslation(),
            TurretConstants.turretCenterPose.getRotation().rotateBy(turretRotation));

        return rotatedTurret.transformBy(new Transform3d(
            TurretConstants.CAMERA_POSE3D.getTranslation(),
            TurretConstants.CAMERA_POSE3D.getRotation()));
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        SwerveDriveState state = swerveState.get();
        Rotation3d gyroRot = gyroRotation.get();
        Rotation2d fieldYaw = state.Pose.getRotation();
        Rotation3d seedRotation = new Rotation3d(gyroRot.getX(), gyroRot.getY(), fieldYaw.getRadians());

        now = Timer.getFPGATimestamp();
        chassisOmegaBuffer.addSample(now, state.Speeds.omegaRadiansPerSecond);
        rollBuffer.addSample(now, gyroRot.getX());
        pitchBuffer.addSample(now, gyroRot.getY());

        Rotation2d turretAngle = Rotation2d.fromDegrees(turretAngleSupplier.get());
        now = Timer.getFPGATimestamp();
        turretAngleBuffer.addSample(now, turretAngle);

        Pose3d robotToCamera = solveRobotToCamera(turretAngle.getDegrees());
        limelightVision.updateTurretCameraOffset(robotToCamera);

        LimelightHelpers.Flush();
        now = Timer.getFPGATimestamp();

        pushedAngleBuffer.addSample(now, turretAngle);
        robotToCameraPublisher.accept(robotToCamera);
        publishTurretCameraDebug(turretAngle.getDegrees(), robotToCamera, state.Pose);

        EstimationMode mode = EstimationMode.MEGATAG1;
        limelightVision.periodic(mode);

        boolean hasTurretCameraEstimate = hasTurretCameraEstimate();
        int accepted = acceptLimelightEstimates(now);

        if (photonVision != null && !hasTurretCameraEstimate) {
            accepted += acceptPhotonEstimates();
            photonVision.periodic();
        }

        headingSeededPublisher.accept(headingSeeded);
        seededPosePublisher.accept(new Pose3d(Translation3d.kZero, seedRotation));
        acceptedCountPublisher.accept(accepted);
        mismatchPublisher.accept(lastMismatchDeg);
        estimateAgePublisher.accept(lastAgeSeconds);
        rejectReasonPublisher.accept(lastRejectReason);
    }

    private int acceptLimelightEstimates(double now) {
        int accepted = 0;

        for (AprilTagEstimate estimate : limelightVision.getAllEstimates()) {
            boolean isTurretCam = limelightVision.isTurretCameraEstimate(estimate);
            now = Timer.getFPGATimestamp();

            if (isTurretCam) {
                lastAgeSeconds = now - estimate.timestampSeconds();
                if (isFresh(estimate.timestampSeconds(), now)) {
                    turretMismatchDeg(estimate.timestampSeconds())
                        .ifPresent(mismatch -> lastMismatchDeg = mismatch);
                }
            }

            String reject = rejectReason(estimate, now, isTurretCam);
            if (!reject.isEmpty()) {
                lastRejectReason = reject;
                continue;
            }

            addVisionMeasurement.accept(estimate.estimatedPose(), estimate.timestampSeconds(),
                DriverStation.isEnabled() ? getStdDevs(estimate) : getDisabledStdDevs(estimate));
            accepted++;
            lastRejectReason = "";
            headingSeeded = true;
        }

        return accepted;
    }

    private boolean hasTurretCameraEstimate() {
        for (AprilTagEstimate estimate : limelightVision.getAllEstimates()) {
            if (limelightVision.isTurretCameraEstimate(estimate)) {
                return true;
            }
        }
        return false;
    }

    private int acceptPhotonEstimates() {
        int accepted = 0;

        for (var estimate : photonVision.getLatestEstimates()) {
            if (estimate.targetsUsed.isEmpty()) continue;

            addVisionMeasurement.accept(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds,
                photonVision.getEstimationStdDevs(estimate.targetsUsed));
            accepted++;
            headingSeeded = true;
        }

        return accepted;
    }

    private String rejectReason(AprilTagEstimate est, double now, boolean isTurretCam) {
        double captureTime = est.timestampSeconds();
        if (now - captureTime < -ATVisionConstants.kClockSkewToleranceSeconds) return "timestamp in the future";
        if (!isFresh(captureTime, now)) return "estimate too old";

        Optional<Double> chassisOmega = chassisOmegaBuffer.getSample(captureTime);
        Optional<Double> roll = rollBuffer.getSample(captureTime);
        Optional<Double> pitch = pitchBuffer.getSample(captureTime);
        if (chassisOmega.isEmpty() || roll.isEmpty() || pitch.isEmpty()) return "no chassis history";
        if (Math.abs(chassisOmega.get()) >= ATVisionConstants.kMaxChassisOmegaRadPerSec) return "chassis yaw rate";
        if (Math.PI-Math.abs(roll.get()) >= ATVisionConstants.kMaxTiltRad
            || Math.abs(pitch.get()) >= ATVisionConstants.kMaxTiltRad) return "robot tilted";

        if (est.tagCount() <= 0) return "no tags";
        if (est.avgAmbiguity() > ATVisionConstants.kMaxAmbiguity) return "ambiguity";
        if (est.estimatedPose().getTranslation().getDistance(Translation2d.kZero) < 0.05) return "pose at origin";

        if (!isTurretCam) return "";

        var mismatch = turretMismatchDeg(captureTime);
        var velocity = turretVelAtCaptureDegPerSec(captureTime);
        if (mismatch.isEmpty() || velocity.isEmpty()) return "no turret history";

        if (mismatch.getAsDouble() > ATVisionConstants.kMaxTurretMismatchDeg) return "turret geometry mismatch";
        if (Math.abs(velocity.getAsDouble()) > ATVisionConstants.kMaxTurretVelDegPerSec) return "turret slewing";

        return "";
    }

    private boolean isFresh(double captureTime, double now) {
        double age = now - captureTime;
        return age >= -ATVisionConstants.kClockSkewToleranceSeconds
            && age <= ATVisionConstants.kMaxEstimateAgeSeconds;
    }

    private OptionalDouble turretMismatchDeg(double captureTime) {
        Optional<Rotation2d> actual = turretAngleBuffer.getSample(captureTime);
        Optional<Rotation2d> used = pushedAngleBuffer.getSample(
            captureTime - ATVisionConstants.kOffsetTransportLatencySeconds);
        if (actual.isEmpty() || used.isEmpty()) return OptionalDouble.empty();
        return OptionalDouble.of(Math.abs(actual.get().minus(used.get()).getDegrees()));
    }

    private OptionalDouble turretVelAtCaptureDegPerSec(double captureTime) {
        Optional<Rotation2d> actual = turretAngleBuffer.getSample(captureTime);
        Optional<Rotation2d> prior = turretAngleBuffer.getSample(
            captureTime - ATVisionConstants.kVelocitySampleDtSeconds);
        if (actual.isEmpty() || prior.isEmpty()) return OptionalDouble.empty();
        return OptionalDouble.of(actual.get().minus(prior.get()).getDegrees()
            / ATVisionConstants.kVelocitySampleDtSeconds);
    }

    public Command captureLimelightRewindsCommand(double seconds) {
        return Commands.runOnce(() -> limelightVision.captureRewinds(seconds), this);
    }

    public static Matrix<N3, N1> getStdDevs(AprilTagEstimate estimate) {
        return estimate.isMegaTag2() ? getStdDevsMT2(estimate) : getStdDevsMT1(estimate);
    }

    public static Matrix<N3, N1> getDisabledStdDevs(AprilTagEstimate estimate) {
        return VecBuilder.fill(
                0.01,
                0.01,
                estimate.isMegaTag2() ? Double.POSITIVE_INFINITY : 0.01);
    }

    private static Matrix<N3, N1> getStdDevsMT2(AprilTagEstimate estimate) {
        double xydevs = ATVisionConstants.kLimelightMT2StdDevCoefficients[0] * stdDevScale(estimate);
        return VecBuilder.fill(
                xydevs,
                xydevs,
                Double.POSITIVE_INFINITY);
    }

    private static Matrix<N3, N1> getStdDevsMT1(AprilTagEstimate estimate) {
        double scale = stdDevScale(estimate);
        return VecBuilder.fill(
                ATVisionConstants.kLimelightMT1StdDevCoefficients[0] * scale,
                ATVisionConstants.kLimelightMT1StdDevCoefficients[0] * scale,
                ATVisionConstants.kLimelightMT1StdDevCoefficients[1] * scale);
    }

    private static double stdDevScale(AprilTagEstimate estimate) {
        if (estimate.tagCount() <= 0) return Double.POSITIVE_INFINITY;
        double distMeters = Math.max(estimate.avgTagDistMeters(), 1.0);
        return (distMeters * distMeters) * ATVisionConstants.kOptimalTagCount / estimate.tagCount();
    }
}

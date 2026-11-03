package frc.robot.subsystems.vision.apriltag;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.apriltag.RealATLimelightVision.LimelightConstants;
import frc.robot.util.LimelightHelpers;

public class AprilTagModule {

    public static record AprilTagEstimate(Pose2d estimatedPose, double timestampSeconds, boolean isMegaTag2, double avgTagDistMeters, int tagCount, double avgAmbiguity, double tagArea) {}

    public enum EstimationMode {
        MEGATAG1,
        MEGATAG2;
    }

    private final String limelightID;

    private final NetworkTable moduleSubTable;

    private final StructPublisher<Pose3d> robotToCameraPublisher;
    private final StructPublisher<Pose3d> estimatePublisher;
    private final BooleanPublisher isActivePublisher;
    private final BooleanPublisher isConnectedPublisher;
    private final StringPublisher modePublisher;
    private final StringPublisher defaultModePublisher;

    private final DoublePublisher ntClockOffsetPublisher;

    private record ReceivedPose(double[] value, double rioReceiveSeconds, double ntTimestampSeconds) {}

    private final AtomicReference<ReceivedPose> latestMT1 = new AtomicReference<>();
    private final AtomicReference<ReceivedPose> latestMT2 = new AtomicReference<>();

    private final EstimationMode defaultMode;
    private EstimationMode lastMode;

    private double lastHb = Double.NaN;
    private double lastHbChangeTimestampSeconds;
    private boolean connected = false;
    private boolean wasConnected = false;
    private double lastDisconnectReportTimestampSeconds = Double.NEGATIVE_INFINITY;

    private Optional<AprilTagEstimate> latestEstimate = Optional.empty();

    public AprilTagModule(String limelightID, Pose3d offset, NetworkTable visionTable) {
        this.limelightID = limelightID;

        defaultMode = LimelightConstants.kDefaultMode;
        lastMode = defaultMode;
        lastHbChangeTimestampSeconds = Timer.getFPGATimestamp();

        moduleSubTable = visionTable.getSubTable(limelightID);
        robotToCameraPublisher = moduleSubTable.getStructTopic("CameraOffset", Pose3d.struct).publish();
        estimatePublisher = moduleSubTable.getStructTopic("PoseEstimate", Pose3d.struct).publish();
        isActivePublisher = moduleSubTable.getBooleanTopic("IsActive").publish();
        modePublisher = moduleSubTable.getStringTopic("LastEstimateMode").publish();
        defaultModePublisher = moduleSubTable.getStringTopic("DefaultEstimateMode").publish();
        isConnectedPublisher = moduleSubTable.getBooleanTopic("IsConnected").publish();
        ntClockOffsetPublisher = moduleSubTable.getDoubleTopic("NTClockOffsetMs").publish();

        listenForPoses("botpose_wpiblue", latestMT1);
        listenForPoses("botpose_orb_wpiblue", latestMT2);

        applyConfig();
        updateOffset(offset);
        defaultModePublisher.setDefault(defaultMode.name());
    }

    private void listenForPoses(String key, AtomicReference<ReceivedPose> latest) {
        DoubleArrayEntry entry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightID, key);
        NetworkTableInstance.getDefault().addListener(
            entry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                double receivedAt = Timer.getFPGATimestamp();
                var value = event.valueData.value;
                latest.set(new ReceivedPose(value.getDoubleArray(), receivedAt, value.getTime() / 1e6));
            });
    }

    private void applyConfig() {

        LimelightHelpers.SetIMUMode(limelightID, 0);

        LimelightHelpers.setRewindEnabled(limelightID, true);
    }

    public void periodic(EstimationMode mode) {
        double now = Timer.getFPGATimestamp();

        double hb = LimelightHelpers.getHeartbeat(limelightID);
        if (hb != lastHb) {
            lastHb = hb;
            lastHbChangeTimestampSeconds = now;
        }

        connected = (now - lastHbChangeTimestampSeconds) < LimelightConstants.kHeartbeatTimeoutSeconds;

        if (connected && !wasConnected) {
            applyConfig();
        }
        wasConnected = connected;

        if (!connected && (now - lastDisconnectReportTimestampSeconds) >= LimelightConstants.kDisconnectReportPeriodSeconds) {
            lastDisconnectReportTimestampSeconds = now;
            DriverStation.reportError(limelightID + " is not connected.", false);
        }

        latestEstimate = readPose(mode == EstimationMode.MEGATAG2);

        updateTelemetry();
    }

    public Optional<AprilTagEstimate> getLatestEstimate() {
        return latestEstimate;
    }

    private void updateTelemetry() {
        estimatePublisher.accept(latestEstimate.map(e -> new Pose3d(e.estimatedPose())).orElse(Pose3d.kZero));
        isActivePublisher.accept(latestEstimate.isPresent());
        modePublisher.accept(lastMode.toString());
        defaultModePublisher.accept(defaultMode.toString());
        isConnectedPublisher.accept(connected);
    }

    public Pose3d getOffset() {
        return LimelightHelpers.getCameraPose3d_RobotSpace(limelightID);
    }

    public void updateOffset(Pose3d offset) {
        Rotation3d cameraRot = offset.getRotation();
        double handedness = LimelightConstants.kLimelightRobotSpaceIsYRight ? -1.0 : 1.0;
        double yawSign = LimelightConstants.kLimelightYawIsInverted ? -1.0 : 1.0;

        LimelightHelpers.setCameraPose_RobotSpace(
            limelightID,
            offset.getX(),
            handedness * offset.getY(),
            offset.getZ(),
            handedness * Math.toDegrees(cameraRot.getX()),
            Math.toDegrees(cameraRot.getY()),
            yawSign * handedness * Math.toDegrees(cameraRot.getZ())
        );
        robotToCameraPublisher.accept(offset);
    }

    public boolean isConnected() {
        return connected;
    }

    public boolean hasTargets() {
        return LimelightHelpers.getTV(limelightID);
    }

    private Optional<AprilTagEstimate> readPose(boolean isMegaTag2) {
        ReceivedPose received = (isMegaTag2 ? latestMT2 : latestMT1).get();
        if (received == null || received.value().length == 0) {

            return Optional.empty();
        }
        double[] poseArray = received.value();

        double latencyMs = LimelightHelpers.extractArrayEntry(poseArray, 6);
        int tagCount = (int) LimelightHelpers.extractArrayEntry(poseArray, 7);

        double tagDistMeters = LimelightHelpers.extractArrayEntry(poseArray, 9);
        double tagArea = LimelightHelpers.extractArrayEntry(poseArray, 10);

        if (tagCount <= 0) {
            return Optional.empty();
        }

        final int valsPerFiducial = 7;
        if (poseArray.length != 11 + valsPerFiducial * tagCount) {

            return Optional.empty();
        }

        double avgAmbiguity = 0;
        for (int i = 0; i < tagCount; i++) {
            int baseIndex = 11 + (i * valsPerFiducial);
            avgAmbiguity += poseArray[baseIndex + 6];
        }
        avgAmbiguity /= tagCount;

        var pose = LimelightHelpers.toPose2D(poseArray);

        double adjustedTimestampSeconds = received.rioReceiveSeconds() - (latencyMs / 1000.0);

        ntClockOffsetPublisher.accept((received.ntTimestampSeconds() - received.rioReceiveSeconds()) * 1000.0);

        lastMode = isMegaTag2 ? EstimationMode.MEGATAG2 : EstimationMode.MEGATAG1;
        return Optional.of(new AprilTagEstimate(pose, adjustedTimestampSeconds, isMegaTag2, tagDistMeters, tagCount, avgAmbiguity, tagArea));
    }

    public void seedOrientation(Rotation3d rot) {

        LimelightHelpers.SetRobotOrientation_NoFlush(
            limelightID,
            Math.toDegrees(rot.getZ()),
            0,
            Math.toDegrees(rot.getY()),
            0,
            Math.toDegrees(rot.getX()),
            0
        );
    }

    public void captureRewind(double seconds) {
        LimelightHelpers.triggerRewindCapture(limelightID, seconds);
    }
}

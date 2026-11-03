package frc.robot.autos;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CANRange;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.turret.Turret.UnwindTarget;
import frc.robot.util.AllianceFlipUtil;

public class Autos {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Superstructure m_superstructure;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;
    private final CANRange m_canRange = new CANRange();

    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());
    public final Supplier<Pose2d> TRENCH_START_LEFT = AllianceFlipUtil.flipped(Depot1Path.getStartPose());
    private final Path Depot2Path = new Path("depot-2".toLowerCase());
    private final Path Depot3Path = new Path("depot-3".toLowerCase());
    private final Path RightDepot1Path = mirroredCopy(Depot1Path);
    private final Path RightDepot2Path = mirroredCopy(Depot2Path);
    private final Path RightDepot3Path = mirroredCopy(Depot3Path);
    private final Supplier<Pose2d> TRENCH_START_RIGHT_DEPOT =
        AllianceFlipUtil.flipped(RightDepot1Path.getStartPose());

    private final Path LeftBump1Path = new Path("path-1".toLowerCase());
    private final Path LeftBump2Path = new Path("path-2".toLowerCase());
    private final Path RightBump1Path = mirroredCopy(LeftBump1Path);
    private final Path RightBump2Path = mirroredCopy(LeftBump2Path);
    private final Supplier<Pose2d> TRENCH_START_RIGHT =
        AllianceFlipUtil.flipped(RightBump1Path.getStartPose());

    private final Path Testcanrange = new Path("Test-canrange".toLowerCase());
    private final Path Testcanrange2 = new Path("Test-canrange2".toLowerCase());

    private final Supplier<Pose2d> TEST_START_CANRANGE = AllianceFlipUtil.flipped(Testcanrange.getStartPose());

    public Autos(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.m_drivetrain = drivetrain;
        this.m_superstructure = superstructure;

        FollowPath.registerEventTrigger("Unwind", m_superstructure.requestTurretUnwind(UnwindTarget.LOWER));

        FollowPath.registerEventTrigger("outtakeEvent",
            Commands.sequence( m_superstructure.requestIntakeEject(),
            Commands.waitSeconds(0.5),
            m_superstructure.requestIntakeIdle()));

        FollowPath.registerEventTrigger("startShooting", m_superstructure.requestRobotScoring());

        FollowPath.registerEventTrigger("startIntakingAgain", m_superstructure.requestIntakeMiniAgitate());

        FollowPath.registerEventTrigger("pauseScoring", Commands.sequence(
            m_superstructure.requestRobotIdle(),
            Commands.waitSeconds(2),
            m_superstructure.requestRobotScoring()
        ));

        FollowPath.registerEventTrigger("stopScoring",
                Commands.parallel(m_superstructure.requestRobotIdle(), superstructure.requestIntakeActive()));

        FollowPath.registerEventTrigger("leftBumpStopIntake", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStartShooter", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("leftBumpStartShooting", m_superstructure.requestRobotScoring());
        FollowPath.registerEventTrigger("leftBumpStartAgitating", m_superstructure.requestIntakeMiniAgitate());
        FollowPath.registerEventTrigger("leftBumpStopAgitating", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStopShooting", m_superstructure.requestRobotIdle());

        FollowPath.registerEventTrigger("leftBumpStopIntakeAgain", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStartShooterAgain", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("leftBumpStartShootingAgain", m_superstructure.requestRobotScoring());
        FollowPath.registerEventTrigger("leftBumpStartAgitatingAgain", m_superstructure.requestIntakeMiniAgitate());

        pathBuilder = new FollowPath.Builder(
                m_drivetrain,
                m_drivetrain::getPose,
                m_drivetrain::getChassisSpeeds,
                m_drivetrain::drive,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(7.0, 0.0, 0.0),
                new PIDController(0.0, 0.0, 0.0)
        )
                .withDefaultShouldFlip();

        registerAutos();
    }

    private void registerAutos() {

        autos.put("(Left) 1 Swipe + Depot",
                () -> auto(TRENCH_START_LEFT.get(), c -> {
                    Command Depot1 = pathBuilder.build(Depot1Path);
                    Command Depot2 = pathBuilder.build(Depot2Path);
                    Command Depot3 = pathBuilder.build(Depot3Path);

                    c.addCommands(untilCloseToWallAfterEvent(Depot1, "hubSensorActivation", 4)
                            .alongWith(m_superstructure.requestIntakeActive()));

                    c.addCommands(untilCloseToWallAfterEvent(Depot2, "depotSensorActivation", 9));

                    c.addCommands(Depot3);
                }));

        autos.put("(Right) 1 Swipe (Depot mirror)",
                () -> auto(TRENCH_START_RIGHT_DEPOT.get(), c -> {
                    Command Depot1 = pathBuilder.build(RightDepot1Path);
                    Command Depot2 = pathBuilder.build(RightDepot2Path);
                    Command Depot3 = pathBuilder.build(RightDepot3Path);

                    c.addCommands(untilCloseToWallAfterEvent(Depot1, "hubSensorActivation", 8)
                            .alongWith(m_superstructure.requestIntakeActive()));

                    c.addCommands(untilCloseToWallAfterEvent(Depot2, "depotSensorActivation", 12));

                    c.addCommands(Depot3);
                }));

        autos.forEach(autoChooser::addCmd);
    }

    private Command untilCloseToWallAfterEvent(Command path, String eventKey, double timeoutSeconds) {
        AtomicBoolean eventFired = new AtomicBoolean(false);
        FollowPath.registerEventTrigger(eventKey, () -> eventFired.set(true));

        return Commands.sequence(

                Commands.runOnce(() -> eventFired.set(false)),

                path.withTimeout(timeoutSeconds)
                        .until(() -> eventFired.get() && m_canRange.isCloseToWall()));
    }

    public void warmUpPathFollowing() {
        try {
            Path warmupPath = new Path(
                    new Path.Waypoint(new Pose2d(2.0, 2.0, Rotation2d.kZero)),
                    new Path.TranslationTarget(3.0, 2.5),
                    new Path.Waypoint(new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(90))));
            for (boolean flip : new boolean[] { false, true }) {
                FollowPath follower = new FollowPath.Builder(
                        m_drivetrain,
                        m_drivetrain::getPose,
                        m_drivetrain::getChassisSpeeds,
                        speeds -> {},
                        new PIDController(5.0, 0.0, 0.0),
                        new PIDController(7.0, 0.0, 0.0),
                        new PIDController(0.0, 0.0, 0.0))
                        .withShouldFlip(() -> flip)
                        .build(warmupPath);
                follower.initialize();
                for (int i = 0; i < 5; i++) {
                    follower.execute();
                    follower.isFinished();
                }
                follower.end(true);
            }
            m_drivetrain.drive(new ChassisSpeeds());
        } catch (RuntimeException e) {

            DriverStation.reportWarning("BLine path-following warmup failed: " + e, false);
        }
    }

    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    public AutoChooser getAutoChooser() {
        return autoChooser;
    }

    public CANRange getCanRange() {
        return m_canRange;
    }

    private static Path mirroredCopy(Path source) {
        Path mirrored = source.copy();
        mirrored.mirror();
        return mirrored;
    }

    private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        group.addCommands(Commands.runOnce(() -> m_drivetrain.resetPose(startPose), m_drivetrain));
        builder.accept(group);
        return group;
    }

    private Command auto(Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        builder.accept(group);
        return group;
    }
}

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.ATVision;
import frc.robot.subsystems.vision.apriltag.NoneATLimelightVision;
import frc.robot.subsystems.vision.apriltag.RealATLimelightVision;
import frc.robot.subsystems.vision.photon.RealPhotonATVision;
import frc.robot.util.logging.HootLogging;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Telemetry;
import frc.robot.util.currentlimit.CurrentLimitManager;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry();
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

    public Superstructure m_superstructure = new Superstructure(m_drivetrain::state);

    public final ATVision m_vision = new ATVision(
            Utils.isSimulation()
                ? new NoneATLimelightVision()
                : new RealATLimelightVision(NetworkTableInstance.getDefault().getTable(ATVision.ATVisionConstants.NT_TABLE)),
            Utils.isSimulation()
                ? null
                : new RealPhotonATVision(NetworkTableInstance.getDefault().getTable(ATVision.ATVisionConstants.NT_TABLE)),
            m_drivetrain::state,
            m_drivetrain.getPigeon2()::getRotation3d,
            m_drivetrain::addVisionMeasurement,
            m_superstructure.m_turret::getAngle);

    private Mechanism2d VISUALIZER;
    public final Autos autos = new Autos(m_drivetrain, m_superstructure);

    public final CurrentLimitManager currentLimitManager = new CurrentLimitManager();

    public final HootLogging hootLogging = new HootLogging(m_superstructure, m_drivetrain);

    public RobotContainer() {
        VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
        SmartDashboard.putData("Visualizer", VISUALIZER);
        SmartDashboard.putData("Auto Mode", autos.getAutoChooser());
        SmartDashboard.putString("Superstructure state", m_superstructure.getRobotState().toString());

        RobotCurrentLimits.configure(currentLimitManager, m_superstructure, m_drivetrain);

        configureBindings();
        RobotVisualizer.setupVisualizer();
        warmUpElastic();
        autos.warmUpPathFollowing();
    }

    private void warmUpElastic() {
        Elastic.selectTab("Warmup");
    }

    private void configureBindings() {
        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        ));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        Trigger ejectButton = joystick.rightBumper();
        ejectButton.onTrue(m_superstructure.requestIntakeEject());

        Trigger shootButton = joystick.rightTrigger();
        Trigger intakeButton = joystick.leftTrigger();
        Trigger notEjecting = ejectButton.negate();
        Trigger shootAndIntake = shootButton.and(intakeButton).and(notEjecting);
        Trigger shootOnly = shootButton.and(intakeButton.negate()).and(notEjecting);
        Trigger intakeOnly = intakeButton.and(shootButton.negate()).and(notEjecting);

        shootButton.onTrue(m_superstructure.requestRobotShooting());
        intakeOnly.onTrue(m_superstructure.requestIntakeActive());
        shootOnly.onTrue(m_superstructure.requestIntakeFullAgitate());
        shootAndIntake.onTrue(m_superstructure.requestIntakeForShootAndIntake());

        shootButton.or(intakeButton).or(ejectButton).negate().onTrue(m_superstructure.requestIntakeIdle());

        joystick.start().and(RobotModeTriggers.disabled()).onTrue(m_superstructure.requestHomeMechanisms());

        joystick.povDown().onTrue(Commands.runOnce(this:: saveLimelightRewind)
            .ignoringDisable(true));

        joystick.y().onTrue(
            m_superstructure.requestRobotShootSafe());

        shootButton.or(joystick.y()).onFalse(m_superstructure.requestRobotIdle());

        joystick.x().whileTrue(m_drivetrain.applyRequest(() -> brake));

    }

    public void saveLimelightRewind() {
        if (!Utils.isSimulation()) {
            for (String cameraName : RealATLimelightVision.LimelightConstants.LL_IDS) {
                LimelightHelpers.triggerRewindCapture(cameraName, 165.0);
            }
            LimelightHelpers.Flush();
        }
    }

    public Command getAutonomousCommand() {
        return autos.selectedCommand();
    }
}

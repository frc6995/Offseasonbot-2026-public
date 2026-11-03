package frc.robot.util.logging;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.CtreUtil;

public final class HootLogging {
    public static final boolean kEnabled = true;

    private static final String kUsbLogPath = "/u/logs";

    private final Superstructure m_superstructure;
    private final CommandSwerveDrivetrain m_swerve;

    private Enum<?> m_lastRobotState;
    private Enum<?> m_lastFlywheelState;
    private Enum<?> m_lastIntakeState;
    private Enum<?> m_lastTurretState;
    private boolean m_lastBrownedOut;

    public HootLogging(Superstructure superstructure, CommandSwerveDrivetrain swerve) {
        m_superstructure = superstructure;
        m_swerve = swerve;

        SignalLogger.enableAutoLogging(false);
        if (!kEnabled) {
            return;
        }

        if (RobotBase.isReal() && new File("/u").isDirectory()) {
            CtreUtil.reportIfNotOk("hoot set path", SignalLogger.setPath(kUsbLogPath));
        }
        CtreUtil.reportIfNotOk("hoot logging start", SignalLogger.start());

        m_lastBrownedOut = RobotController.isBrownedOut();
        SignalLogger.writeBoolean("BrownedOut", m_lastBrownedOut);
    }

    public void periodic() {
        if (!kEnabled) {
            return;
        }

        boolean brownedOut = RobotController.isBrownedOut();
        if (brownedOut != m_lastBrownedOut) {
            SignalLogger.writeBoolean("BrownedOut", brownedOut);
            m_lastBrownedOut = brownedOut;
        }

        SignalLogger.writeStruct("Pose", Pose2d.struct, m_swerve.getState().Pose);

        Enum<?> robotState = m_superstructure.getRobotState();
        if (robotState != m_lastRobotState) {
            SignalLogger.writeString("Robot State", robotState.name());
            m_lastRobotState = robotState;
        }
        Enum<?> flywheelState = m_superstructure.m_flywheel.getState();
        if (flywheelState != m_lastFlywheelState) {
            SignalLogger.writeString("Flywheel/State", flywheelState.name());
            m_lastFlywheelState = flywheelState;
        }
        Enum<?> intakeState = m_superstructure.m_intake.getState();
        if (intakeState != m_lastIntakeState) {
            SignalLogger.writeString("Intake/State", intakeState.name());
            m_lastIntakeState = intakeState;
        }

        Enum<?> turretState = m_superstructure.m_turret.getState();
        if (turretState != m_lastTurretState) {
            SignalLogger.writeString("Turret/State", turretState.name());
            m_lastTurretState = turretState;
        }
    }
}

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.intake.Intake.IntakeConstants;
import frc.robot.util.currentlimit.CurrentLimit;
import frc.robot.util.currentlimit.CurrentLimitManager;

public final class RobotCurrentLimits {
    private RobotCurrentLimits() {}

    private static final CurrentLimit kShootingRollerLimit = new CurrentLimit(40, 20);
    private static final CurrentLimit kShootingKickerLimit = new CurrentLimit(40, 20);
    private static final CurrentLimit kShootingExtensionLimit = new CurrentLimit(13, 20);
    private static final CurrentLimit kShootingDriveLimit = CurrentLimit.supplyOnly(10);

    private static final CurrentLimit kTeleopDriveLimit =
            CurrentLimit.supplyOnly(TunerConstants.kDriveNominalSupplyCurrentLimitAmps);
    private static final CurrentLimit kAutoDriveLimit = CurrentLimit.supplyOnly(50);

    public static void configure(
            CurrentLimitManager manager,
            Superstructure superstructure,
            CommandSwerveDrivetrain drivetrain) {

        manager.registerTarget(
                "Intake/Roller",
                new CurrentLimit(IntakeConstants.kRollerStatorCurrentLimitAmps, IntakeConstants.kRollerSupplyCurrentLimitAmps),
                superstructure.m_intake::setRollerCurrentLimit);

        manager.registerTarget(
                "Intake/Kicker",
                new CurrentLimit(IntakeConstants.kKickerStatorCurrentLimitAmps, IntakeConstants.kKickerSupplyCurrentLimitAmps),
                superstructure.m_intake::setKickerCurrentLimit);

        manager.registerTarget(
                "Intake/Extension",
                new CurrentLimit(IntakeConstants.kExtensionStatorCurrentLimitAmps, IntakeConstants.kExtensionSupplyCurrentLimitAmps),
                superstructure.m_intake::setExtensionCurrentLimit);

        manager.registerTarget(
                "Drivetrain/Drive",
                () -> DriverStation.isAutonomousEnabled() ? kAutoDriveLimit : kTeleopDriveLimit,
                limit -> drivetrain.setDriveSupplyCurrentLimit(limit.supplyCurrentLimitAmps()));

        manager.addRule(
                () -> {
                    RobotState state = superstructure.getRobotState();
                    return state == RobotState.SCORING || state == RobotState.PASSING
                            || state == RobotState.SAFE_SHOT;
                },
                Map.of(
                        "Intake/Roller", kShootingRollerLimit,
                        "Intake/Kicker", kShootingKickerLimit,
                        "Intake/Extension", kShootingExtensionLimit,
                        "Drivetrain/Drive", kShootingDriveLimit));
    }
}

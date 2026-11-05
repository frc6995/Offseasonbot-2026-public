package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Elastic;
import frc.robot.util.LoopTiming;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private double autoSimTime = 20.0;

    private final RobotContainer m_robotContainer;

    public Robot() {

        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();

        m_robotContainer = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {
        LoopTiming.getInstance().start();

        CommandScheduler.getInstance().run();
        m_robotContainer.hootLogging.periodic();

        LoopTiming.getInstance().end();

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {

        m_robotContainer.currentLimitManager.setEnabled(false);

        if (RobotBase.isSimulation()) {
            CommandScheduler.getInstance().schedule(
                    Commands.waitSeconds(autoSimTime)
                            .andThen(
                                    () -> {
                                        DriverStationSim.setEnabled(false);
                                        DriverStationSim.notifyNewData();
                                    })
                            .onlyWhile(DriverStation::isAutonomousEnabled));
        }
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        Elastic.selectTab("Autonomous");
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        m_robotContainer.currentLimitManager.setEnabled(true);

        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        m_robotContainer.saveLimelightRewind();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {

        RobotVisualizer.publish();
    }
}

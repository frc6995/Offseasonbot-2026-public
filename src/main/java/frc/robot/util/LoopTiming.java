package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class LoopTiming {

    private static final double PERIOD_MS = 20.0;

    private static final int WINDOW_LOOPS = 250;

    private static final LoopTiming INSTANCE = new LoopTiming();

    private final DoublePublisher m_currentMs;
    private final DoublePublisher m_minMs;
    private final DoublePublisher m_maxMs;
    private final DoublePublisher m_avgMs;
    private final DoublePublisher m_overruns;

    private long m_startUs;
    private double m_minMsValue = Double.MAX_VALUE;
    private double m_maxMsValue = 0.0;
    private double m_sumMs = 0.0;
    private long m_count = 0;
    private long m_overrunCount = 0;

    private LoopTiming() {
        var table = NetworkTableInstance.getDefault().getTable("LoopTiming");
        m_currentMs = table.getDoubleTopic("currentMs").publish();
        m_minMs = table.getDoubleTopic("minMs").publish();
        m_maxMs = table.getDoubleTopic("maxMs").publish();
        m_avgMs = table.getDoubleTopic("avgMs").publish();
        m_overruns = table.getDoubleTopic("overrunCount").publish();
    }

    public static LoopTiming getInstance() {
        return INSTANCE;
    }

    public void start() {
        m_startUs = RobotController.getFPGATime();
    }

    public void end() {
        double ms = (RobotController.getFPGATime() - m_startUs) / 1000.0;

        if (ms > PERIOD_MS) {
            m_overrunCount++;
            m_overruns.set(m_overrunCount);
        }

        m_minMsValue = Math.min(m_minMsValue, ms);
        m_maxMsValue = Math.max(m_maxMsValue, ms);
        m_sumMs += ms;
        m_count++;

        m_currentMs.set(ms);
        m_minMs.set(m_minMsValue);
        m_maxMs.set(m_maxMsValue);
        m_avgMs.set(m_sumMs / m_count);

        if (m_count >= WINDOW_LOOPS) {
            m_minMsValue = Double.MAX_VALUE;
            m_maxMsValue = 0.0;
            m_sumMs = 0.0;
            m_count = 0;
        }
    }
}

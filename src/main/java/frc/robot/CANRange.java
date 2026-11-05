package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;

import edu.wpi.first.wpilibj.RobotBase;

public class CANRange {

    public class CANRangeConstants {
        public static final int kCAN_ID = 35;
        public static final double kProximityThreshold = 0.1;
    }

    CANrangeConfiguration m_frontCANrangeConfigurator = new CANrangeConfiguration();

    private BooleanSupplier m_simProximitySupplier = () -> false;

    public CANRange() {
        m_frontCANrangeConfigurator.ProximityParams.ProximityThreshold = CANRangeConstants.kProximityThreshold;

    }

    public void setSimProximitySupplier(BooleanSupplier simProximitySupplier) {
        m_simProximitySupplier = simProximitySupplier;
    }

    public void periodic() {

    }

    public Boolean isCloseToWall() {
        if (RobotBase.isSimulation()) {
            return m_simProximitySupplier.getAsBoolean();
        }
       return false;
    }
}

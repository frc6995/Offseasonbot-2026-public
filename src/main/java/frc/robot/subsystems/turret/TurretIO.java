package frc.robot.subsystems.turret;
public interface TurretIO {
    public default void setAngle(double angle) {};
    public default void resetEncoder() {};
    public default void updateInputs(TurretIOInputs inputs) {};
    public default void disable() {};

    public class TurretIOInputs {
        public double angle;

        public double velocity;
        public double appliedVolts;
        public double statorCurrent;
    }

}

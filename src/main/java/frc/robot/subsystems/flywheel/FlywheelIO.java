package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    default void updateInputs(FlywheelInputs inputs) {}
    default void setVelocityRPM(double velocityRPM) {}
    default void stop() {}

    public static final int kMotorCount = 4;

    public class FlywheelInputs {
        public double velocityRPM;
        public double appliedVolts;

        public double statorCurrentAmps;

        public double[] motorStatorCurrentAmps = new double[kMotorCount];
    }

}

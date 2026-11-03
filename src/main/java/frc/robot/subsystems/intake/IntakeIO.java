package frc.robot.subsystems.intake;

public interface IntakeIO {
    default void updateInputs(IntakeInputs inputs) {}
    default void setRollerVoltage(double voltage) {}
    default void setKickerVoltage(double voltage) {}
    default void setExtensionPosition(double positionMeters) {}

    default void setRollerCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void setKickerCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void setExtensionCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void resetEncoder() {}

    default void stop() {
        setRollerVoltage(0.0);
        setKickerVoltage(0.0);
    }

    int kRollerMotorCount = 2;
    int kExtensionMotorCount = 2;
    int kKickerMotorCount = 1;

    class IntakeInputs {
        public double rollerAppliedVolts;

        public double rollerStatorCurrentAmps;

        public double[] rollerMotorStatorCurrentAmps = new double[kRollerMotorCount];

        public double kickerAppliedVolts;
        public double kickerStatorCurrentAmps;

        public double extensionPositionMeters;
        public double extensionAppliedVolts;

        public double extensionStatorCurrentAmps;

        public double[] extensionMotorStatorCurrentAmps = new double[kExtensionMotorCount];
    }
}

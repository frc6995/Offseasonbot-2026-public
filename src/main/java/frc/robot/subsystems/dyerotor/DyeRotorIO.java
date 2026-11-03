package frc.robot.subsystems.dyerotor;

public interface DyeRotorIO {
  default void updateInputs(DyeRotorInputs inputs) {}
  default void setSpinVelocity(double velocityRPM) {}
  default void setIndexVoltage(double volts) {}
  default void setIndexVelocity(double velocityRPM) {}
  default void stop() {}

  int kIndexMotorCount = 2;

  public class DyeRotorInputs {
    public double spinPositionRotations;
    public double spinVelocityRPM;
    public double spinAppliedVolts;
    public double spinStatorCurrentAmps;

    public double indexPositionRotations;
    public double indexVelocityRPM;
    public double indexAppliedVolts;

    public double indexStatorCurrentAmps;

    public double[] indexMotorStatorCurrentAmps = new double[kIndexMotorCount];
  }
}

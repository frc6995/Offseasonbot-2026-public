package frc.robot.subsystems.hood;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {
    public static class HoodConstants {
        public static final int kCANID = 44;

        public static final double[][] kAngleData = {
// Generated from calculator, settings = launch@hood0=85°, 1°/hood°, aim +0.25, weights 0.35/0.15/0.2, drag off
        { 1.0, 5.75 },
        { 1.5, 11.75 },
        { 2.0, 14.75 },
        { 2.5, 17.25 },
        { 3.0, 19.25 },
        { 3.5, 23.5 },
        { 4.0, 25.25 },
        { 4.5, 29.75 },
        { 5.0, 33.0 },
        { 6.0, 34.0 },
        { 7.0, 35.0 },
        { 8.0, 35.75 },
        { 9.0, 36.0 },
        { 10.0, 36.5 }
};

        public static final double[][] kPassingAngleData = {
                { 4, 15 },
                { 6.5, 25 },
                { 8, 30 },
                { 10, 40}

        };

        public static final double kP = 500;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 6;
        public static final double kG = 0;
        public static final double kA = 0.05;

        public static final double kHoodCruiseVelocityRotationsPerSec = 6;
        public static final double kHoodAccelerationRotationsPerSec2 = 6;

        public static final double kStatorCurrentLimitAmps = 20;
        public static final double kSupplyCurrentLimitAmps = 40;

        public static final double kReduction = 70.2857;

        public static final double kMinAngleDeg = 0;
        public static final double kMaxAngleDeg = 40;

        public static final double kMOI = 0.00336535601;

        public static final double kHoodLengthMeters = 0.141478;

        public static final double kSafeShotAngleDeg = 20.4;

    }

    public enum HoodState {
        DISABLED,
        ACTIVE,
        SAFE_SHOT
    }

    private HoodIO io;
    private HoodIOInputs inputs = new HoodIOInputs();

    private final MechanismLigament2d hoodLigament = new MechanismLigament2d("hood", Units.inchesToMeters(4), 0, 6,
            new Color8Bit(52, 137, 235));

    private double commandedAngleDeg;

    private HoodState hoodState = HoodState.DISABLED;

    private boolean isZeroed = false;

    private final Supplier<Double> targetAngleDeg;

    public Hood(HoodIO io, Supplier<Double> targetAngleDeg) {
        this.io = io;
        this.targetAngleDeg = targetAngleDeg;
        RobotVisualizer.addHood(hoodLigament);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setState(HoodState.DISABLED);
        }

        io.updateInputs(inputs);

        switch (hoodState) {
            case DISABLED:
                io.disable();
                break;
            case ACTIVE:

                commandedAngleDeg = applyLimits(targetAngleDeg.get());
                io.setAngle(commandedAngleDeg);
                break;
            case SAFE_SHOT:
                commandedAngleDeg = applyLimits(HoodConstants.kSafeShotAngleDeg);
                io.setAngle(commandedAngleDeg);
                break;
        }
    }

    @Override
    public void simulationPeriodic() {
        hoodLigament.setAngle(commandedAngleDeg);
        RobotVisualizer.updateHood(Units.degreesToRadians(commandedAngleDeg));
    }

    public void setState(HoodState state) {
        hoodState = state;
    }

    public void requestActive() {
        setState(HoodState.ACTIVE);
    }

    public void requestDisable() {
        setState(HoodState.DISABLED);
    }

    public void resetEncoder() {
        io.resetEncoder();
        isZeroed = true;
    }
    public boolean isZeroed(){
        return isZeroed;
    }

    public double applyLimits(double angle) {
        double clamped = MathUtil.clamp(angle, Hood.HoodConstants.kMinAngleDeg, Hood.HoodConstants.kMaxAngleDeg);

        return clamped;
    }

    public HoodState getState() {
        return hoodState;
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getStatorCurrent() {
        return inputs.statorCurrent;
    }
}

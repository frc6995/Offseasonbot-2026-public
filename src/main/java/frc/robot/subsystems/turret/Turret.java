package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    public static class TurretConstants {
        public static final int kCANID = 45;

        public static final double kP = 400;
        public static final double kI = 0;
        public static final double kD = 2;
        public static final double kS = 0.2;
        public static final double kV = 2;
        public static final double kA = 0.07;

        public static final double kStatorCurrentLimitAmps = 60;
        public static final double kSupplyCurrentLimitAmps = 40;

        public static final double kCentralAngleDeg = -135.612;
        public static final double kMinAngleDeg = -288 + kCentralAngleDeg;
        public static final double kMaxAngleDeg = 288 + kCentralAngleDeg;
        public static final double kSafeShotAngleDeg = 0;
        public static final double kToleranceDeg = 15;

        public static final double kCruiseVelocityDegPerSec = 720;
        public static final double kMaxAccelerationDegPerSec2 = 3600;

        public static final double kReduction = 32.5;
        public static final double kMOI = 0.0873236726;

        public static final double kLengthMeters = 0.1651;

        public static final Pose3d turretCenterPose =  new Pose3d( new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0),Units.inchesToMeters(21.67)), new Rotation3d(0,0,0));

        public static final Pose3d CAMERA_POSE3D = new Pose3d( new Translation3d(Units.inchesToMeters(5.49), Units.inchesToMeters(0),Units.inchesToMeters(0)), new Rotation3d(0,0.366519,0));

        public static final double kEChainBaseWidth = 9.375 * 0.0254;
        public static final double kEChainBaseLength = 11.385 * 0.0254;
        public static final double kNMPerVolt = 48.9;
        public static final double kSpringForceN = 15 * 4.44822162;
        public static final double[] kAltTurretAngles = {-720, -360, 360, 720};
    }

    public enum TurretState {
        DISABLED,
        AIM_CLOSEST,
        AIM_CENTRAL,
        SAFE_SHOT,
        MANUAL,
        UNWIND;
    }

    public enum UnwindTarget {
        CENTRAL,
        LOWER,
        UPPER;
    }

    private TurretState turretState = TurretState.AIM_CLOSEST;
    private UnwindTarget unwindTarget = UnwindTarget.CENTRAL;
    private TurretState turretStateBeforeUnwind = TurretState.AIM_CLOSEST;
    private double requestedAngleDeg = 0;

    private boolean manualUsesCentralAngle = false;
    private boolean isZeroed = false;

    private double commandedAngleDeg = 0;

    private TurretIO io;
    private Supplier<Double> targetAngleDeg;

    private TurretIOInputs inputs = new TurretIOInputs();

    private final MechanismLigament2d turretLigament = new MechanismLigament2d("turret", Units.inchesToMeters(12), 0, 6,
        new Color8Bit(137, 52, 235));

    public Turret(TurretIO io, Supplier<Double> targetAngleDeg) {
        this.io = io;
        this.targetAngleDeg = targetAngleDeg;

        RobotVisualizer.addTurret(turretLigament);
    }

    public void requestAimClosest() {
        turretState = TurretState.AIM_CLOSEST;
    }

    public void requestAimCentral() {
        turretState = TurretState.AIM_CENTRAL;
    }

    public void requestDisable() {
        turretState = TurretState.DISABLED;
    }

    public void requestUnwind(UnwindTarget target) {
        if(turretState != TurretState.UNWIND) turretStateBeforeUnwind = turretState;
        unwindTarget = target;
        turretState = TurretState.UNWIND;
    }

    public void requestUnwind() {
        requestUnwind(UnwindTarget.CENTRAL);
    }

    public void resetEncoder() {
        io.resetEncoder();
        isZeroed = true;
    }

    public boolean isZerod(){
        return isZeroed;
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);

        switch (turretState) {
            case DISABLED -> io.disable();
            case AIM_CENTRAL -> commandedAngleDeg = selectCentralAngle(targetAngleDeg.get());
            case AIM_CLOSEST -> commandedAngleDeg = selectClosestAngle(targetAngleDeg.get());
            case MANUAL -> commandedAngleDeg = manualUsesCentralAngle
                    ? selectCentralAngle(requestedAngleDeg)
                    : selectClosestAngle(requestedAngleDeg);
            case SAFE_SHOT -> commandedAngleDeg = selectClosestAngle(TurretConstants.kSafeShotAngleDeg);
            case UNWIND -> {
                if(unwindTarget == UnwindTarget.CENTRAL) {
                    commandedAngleDeg = selectCentralAngle(targetAngleDeg.get());
                } else if(unwindTarget == UnwindTarget.LOWER) {
                    commandedAngleDeg = selectLowerAngle(targetAngleDeg.get());
                } else if(unwindTarget == UnwindTarget.UPPER) {
                    commandedAngleDeg = selectUpperAngle(targetAngleDeg.get());
                } else
                commandedAngleDeg = selectCentralAngle(targetAngleDeg.get());
            }
        }

        if(turretState == TurretState.UNWIND) {
            if(MathUtil.isNear(commandedAngleDeg, getAngle(), 15.0)) {
                turretState = turretStateBeforeUnwind;
            }
        }

        publishTelemetry();
    }

    private void publishTelemetry() {
        SmartDashboard.putString("Turret/State", turretState.name());
    }

    @Override
    public void simulationPeriodic() {
        turretLigament.setAngle(commandedAngleDeg);
        RobotVisualizer.updateTurret(Units.degreesToRadians(commandedAngleDeg));
    }

    public void setState(TurretState state) {
        turretState = state;
    }

    public void setAngleManual(double angle) {
        requestedAngleDeg = angle;
        manualUsesCentralAngle = false;

        this.turretState = TurretState.MANUAL;
    }

    public void setClosestAngleManual(double angle) {

        requestedAngleDeg = angle;
        manualUsesCentralAngle = false;

        this.turretState = TurretState.MANUAL;
    }

    public void setCentralAngleManual(double angle) {
        requestedAngleDeg = angle;
        manualUsesCentralAngle = true;

        this.turretState = TurretState.MANUAL;
    }

    private double selectClosestAngle(double angle) {
        double currentAngle = this.getAngle();

        angle = MathUtil.inputModulus(angle, -180, 180);

        boolean isValid = withinBounds(angle);

        double smallestAngle = angle;
        double smallestDifference = Math.abs(angle - currentAngle);

        for(double offset : TurretConstants.kAltTurretAngles) {
            double alt = angle + offset;
            double diff = Math.abs(alt - currentAngle);
            if ((diff < smallestDifference || !isValid) && withinBounds(alt)) {
                smallestDifference = diff;
                smallestAngle = alt;
                isValid = true;
            }
        }

        io.setAngle(smallestAngle);
        return smallestAngle;
    }

    private double selectCentralAngle(double angle) {
        angle = MathUtil.inputModulus(angle, -180+TurretConstants.kCentralAngleDeg, 180+TurretConstants.kCentralAngleDeg);

        io.setAngle(angle);
        return angle;
    }

    private double selectLowerAngle(double angle) {
        angle = MathUtil.inputModulus(angle, TurretConstants.kMinAngleDeg, TurretConstants.kMinAngleDeg + 360.0);
        io.setAngle(angle);
        return angle;
    }

    private double selectUpperAngle(double angle) {
        angle = MathUtil.inputModulus(angle, TurretConstants.kMaxAngleDeg - 360.0, TurretConstants.kMaxAngleDeg);
        io.setAngle(angle);
        return angle;
    }

    private boolean withinBounds(double angleDeg) {
        return angleDeg < TurretConstants.kMaxAngleDeg && angleDeg > TurretConstants.kMinAngleDeg;
    }

    public TurretState getState() {
        return turretState;
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getStatorCurrent() {
        return inputs.statorCurrent;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(commandedAngleDeg, getAngle(), TurretConstants.kToleranceDeg);
    }
}

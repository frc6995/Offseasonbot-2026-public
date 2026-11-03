package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.CtreUtil;

import static frc.robot.subsystems.turret.Turret.TurretConstants.*;

public class TurretIOTalonFX implements TurretIO {

    protected final TalonFX m_turretMotor = new TalonFX(kCANID, Constants.CANBuses.UpperBus);

    protected final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    protected StatusSignal<Angle> angleSignal;
    protected StatusSignal<AngularVelocity> velocitySignal;
    protected StatusSignal<Voltage> voltSignal;
    protected StatusSignal<Current> statorCurrentSignal;

    protected double cachedAngle = 0;

    public TurretIOTalonFX() {
        configMotor();

        angleSignal = m_turretMotor.getPosition();
        velocitySignal = m_turretMotor.getVelocity();
        voltSignal = m_turretMotor.getMotorVoltage();

        statorCurrentSignal = m_turretMotor.getStatorCurrent();

        CtreUtil.setCurrentSignalFrequency(statorCurrentSignal);
        CtreUtil.setPowerSignalFrequency(m_turretMotor);

        BaseStatusSignal.setUpdateFrequencyForAll(
                CtreUtil.kMechanismSignalFrequencyHz, angleSignal, velocitySignal, voltSignal);

        CtreUtil.reportIfNotOk("Turret optimize bus utilization",
                ParentDevice.optimizeBusUtilizationForAll(m_turretMotor));
    }

    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        config.CurrentLimits =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(kSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true);

        config.Feedback =
            new FeedbackConfigs()

                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(kReduction);

        config.Slot0 =
            new Slot0Configs()
                .withKP(kP)
                .withKV(kV)
                .withKA(kA)
                .withKS(kS)
                .withKD(kD);

        config.MotionMagic =
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(angleToMechanismRotations(kCruiseVelocityDegPerSec))
                .withMotionMagicAcceleration(angleToMechanismRotations(kMaxAccelerationDegPerSec2));

        config.SoftwareLimitSwitch =
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angleToMechanismRotations(kMaxAngleDeg))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(angleToMechanismRotations(kMinAngleDeg));

        config.HardwareLimitSwitch =
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(false)
                .withReverseLimitEnable(false);

        CtreUtil.reportIfNotOk("Config Turret", m_turretMotor.getConfigurator().apply(config));
    }

    @Override
    public void resetEncoder() {
        m_turretMotor.setPosition(0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, velocitySignal, voltSignal, statorCurrentSignal);

        cachedAngle = angleSignal.getValueAsDouble();

        inputs.angle = mechanismToAngleDegrees(cachedAngle);

        inputs.velocity = mechanismToAngleDegrees(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    }

    @Override
    public void setAngle(double angle) {
        double clampedAngle = MathUtil.clamp(angle, kMinAngleDeg, kMaxAngleDeg);

        double rotations = clampedAngle / 360;

        m_turretMotor.setControl(positionRequest.withPosition(rotations));
    }

    protected double angleToMotorRotations(double angle) {
        return (angle/360)*kReduction;
    }

    protected double angleToMechanismRotations(double angle) {
        return angle / 360.0;
    }

    protected double mechanismToAngleDegrees(double rotations) {
        return rotations * 360.0;
    }

    @Override
    public void disable() {
        m_turretMotor.set(0);
    }
}

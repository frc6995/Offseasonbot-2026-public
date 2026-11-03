package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtil {
    private CtreUtil() {}

    public static final double kCurrentSignalFrequencyHz = 20.0;

    public static final double kMechanismSignalFrequencyHz = 100.0;

    public static void setCurrentSignalFrequency(BaseStatusSignal... signals) {
        reportIfNotOk(
                "set current signal update frequency",
                BaseStatusSignal.setUpdateFrequencyForAll(kCurrentSignalFrequencyHz, signals));
    }

    public static void setPowerSignalFrequency(TalonFX... motors) {
        BaseStatusSignal[] signals = new BaseStatusSignal[motors.length * 2];
        for (int i = 0; i < motors.length; i++) {
            signals[2 * i] = motors[i].getSupplyCurrent(false);
            signals[2 * i + 1] = motors[i].getSupplyVoltage(false);
        }
        reportIfNotOk(
                "set power signal update frequency",
                BaseStatusSignal.setUpdateFrequencyForAll(kCurrentSignalFrequencyHz, signals));
    }

    public static void configureKrakenX60Sim(
            TalonFXSimState simState,
            ChassisReference chassisReference) {
        configureKrakenSim(simState, chassisReference, MotorType.KrakenX60);
    }

    public static void configureKrakenX44Sim(
            TalonFXSimState simState,
            ChassisReference chassisReference) {
        configureKrakenSim(simState, chassisReference, MotorType.KrakenX44);
    }

    private static void configureKrakenSim(
            TalonFXSimState simState,
            ChassisReference chassisReference,
            MotorType motorType) {
        simState.Orientation = chassisReference;
        reportIfNotOk("sim set motor type", simState.setMotorType(motorType));
    }

    public static void reportIfNotOk(String action, StatusCode statusCode) {
        if (!statusCode.isOK()) {
            DriverStation.reportWarning(
                    "CTRE " + action + " returned " + statusCode.getName() + ": "
                            + statusCode.getDescription(),
                    false);
        }
    }
}

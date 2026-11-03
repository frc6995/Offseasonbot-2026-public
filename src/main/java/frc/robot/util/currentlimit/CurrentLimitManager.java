package frc.robot.util.currentlimit;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CurrentLimitManager extends SubsystemBase {

    private static final double kDefaultMinReapplyIntervalSeconds = 0.1;

    private static final class Target {
        final Supplier<CurrentLimit> nominal;
        final Consumer<CurrentLimit> apply;
        CurrentLimit lastApplied;
        double lastAppliedTimestamp = -Double.MAX_VALUE;

        boolean everApplied = false;

        Target(Supplier<CurrentLimit> nominal, Consumer<CurrentLimit> apply) {
            this.nominal = nominal;
            this.apply = apply;
            this.lastApplied = nominal.get();
        }
    }

    private static final class Rule {
        final BooleanSupplier condition;
        final Map<String, CurrentLimit> targetLimits;

        Rule(BooleanSupplier condition, Map<String, CurrentLimit> targetLimits) {
            this.condition = condition;
            this.targetLimits = targetLimits;
        }
    }

    private final double minReapplyIntervalSeconds;
    private final Map<String, Target> targets = new LinkedHashMap<>();
    private final List<Rule> rules = new ArrayList<>();

    private final Map<String, CurrentLimit> desiredScratch = new LinkedHashMap<>();
    private boolean enabled = true;

    private final ExecutorService applyExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "CurrentLimitManager-apply");
        thread.setDaemon(true);
        return thread;
    });

    public CurrentLimitManager() {
        this(kDefaultMinReapplyIntervalSeconds);
    }

    public CurrentLimitManager(double minReapplyIntervalSeconds) {
        this.minReapplyIntervalSeconds = minReapplyIntervalSeconds;
    }

    public void registerTarget(String name, CurrentLimit nominalLimit, Consumer<CurrentLimit> applyLimit) {
        registerTarget(name, () -> nominalLimit, applyLimit);
    }

    public void registerTarget(String name, Supplier<CurrentLimit> nominalLimit, Consumer<CurrentLimit> applyLimit) {
        targets.put(name, new Target(nominalLimit, applyLimit));
    }

    public void addRule(BooleanSupplier condition, Map<String, CurrentLimit> targetLimits) {
        for (String name : targetLimits.keySet()) {
            if (!targets.containsKey(name)) {
                throw new IllegalArgumentException("Unknown current limit target: " + name);
            }
        }
        rules.add(new Rule(condition, Map.copyOf(targetLimits)));
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        Map<String, CurrentLimit> desired = desiredScratch;
        desired.clear();
        if (enabled) {
            for (Rule rule : rules) {
                if (!rule.condition.getAsBoolean()) {
                    continue;
                }
                for (Map.Entry<String, CurrentLimit> entry : rule.targetLimits.entrySet()) {
                    desired.merge(entry.getKey(), entry.getValue(), CurrentLimit::mostRestrictive);
                }
            }
        }

        double now = Timer.getFPGATimestamp();
        for (Map.Entry<String, Target> entry : targets.entrySet()) {
            String name = entry.getKey();
            Target target = entry.getValue();

            CurrentLimit nominal = target.nominal.get();
            CurrentLimit ruled = desired.get(name);
            CurrentLimit want = (ruled == null) ? nominal : ruled.resolvedAgainst(nominal);
            boolean dueForReapply = (now - target.lastAppliedTimestamp) >= minReapplyIntervalSeconds;
            if ((!target.everApplied || !want.equals(target.lastApplied)) && dueForReapply) {

                target.lastApplied = want;
                target.lastAppliedTimestamp = now;
                target.everApplied = true;
                dispatchApply(name, target.apply, want);
            }
        }
    }

    private void dispatchApply(String name, Consumer<CurrentLimit> applyLimit, CurrentLimit want) {
        applyExecutor.execute(() -> {
            try {
                applyLimit.accept(want);
            } catch (RuntimeException e) {
                DriverStation.reportError(
                        "CurrentLimitManager: failed to apply " + want + " to " + name + ": " + e, e.getStackTrace());
            }
        });
    }
}

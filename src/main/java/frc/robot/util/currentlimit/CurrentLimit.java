package frc.robot.util.currentlimit;

public record CurrentLimit(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {

    public static CurrentLimit supplyOnly(double supplyCurrentLimitAmps) {
        return new CurrentLimit(-1.0, supplyCurrentLimitAmps);
    }

    public static CurrentLimit statorOnly(double statorCurrentLimitAmps) {
        return new CurrentLimit(statorCurrentLimitAmps, -1.0);
    }

    public boolean managesStator() {
        return statorCurrentLimitAmps > 0;
    }

    public boolean managesSupply() {
        return supplyCurrentLimitAmps > 0;
    }

    CurrentLimit resolvedAgainst(CurrentLimit fallback) {
        return new CurrentLimit(
                managesStator() ? statorCurrentLimitAmps : fallback.statorCurrentLimitAmps,
                managesSupply() ? supplyCurrentLimitAmps : fallback.supplyCurrentLimitAmps);
    }

    static CurrentLimit mostRestrictive(CurrentLimit a, CurrentLimit b) {
        return new CurrentLimit(
                mostRestrictiveAxis(a.statorCurrentLimitAmps, b.statorCurrentLimitAmps),
                mostRestrictiveAxis(a.supplyCurrentLimitAmps, b.supplyCurrentLimitAmps));
    }

    private static double mostRestrictiveAxis(double a, double b) {
        if (a <= 0) return b;
        if (b <= 0) return a;
        return Math.min(a, b);
    }
}

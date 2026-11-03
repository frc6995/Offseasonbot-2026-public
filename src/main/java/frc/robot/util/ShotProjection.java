package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class ShotProjection {
    public static final class ShotConstants {

        public static final double kShotDelay = 0.02;

        public static final double[][] kTofData = {
                { 1.0, 0.987 }, { 1.5, 0.910 }, { 2.0, 0.970 }, { 2.5, 1.017 }, { 3.0, 1.034 },
                { 3.5, 0.991 }, { 4.0, 0.944 }, { 4.5, 0.995 }, { 5.0, 1.040 }, { 5.5, 1.092 },
                { 6.0, 1.134 }
        };
    }

    public static final class Lut {
        private final double[] x, y;

        public Lut(double[][] data) {
            x = new double[data.length];
            y = new double[data.length];
            for (int i = 0; i < data.length; i++) {
                x[i] = data[i][0];
                y[i] = data[i][1];
            }
        }

        public double get(double d) {
            int n = x.length;
            if (d <= x[0])
                return y[0];
            if (d >= x[n - 1])
                return y[n - 1];
            int lo = 0, hi = n - 1;
            while (hi - lo > 1) {
                int mid = (lo + hi) >>> 1;
                if (x[mid] <= d)
                    lo = mid;
                else
                    hi = mid;
            }
            double f = (d - x[lo]) / (x[hi] - x[lo]);
            return y[lo] + f * (y[hi] - y[lo]);
        }
    }

    private static final int kMaxIterations = 5;
    private static final double kConvergedMeters = 1e-3;

    private final DoublePublisher m_distancePublisher;

    private final Lut tofLut, hoodLut, rpmLut, passRPMLut,passHoodLut;

    public double distance;

    public double robotAngleRad;
    public double hoodDeg;
    public double rpm;
    public double tofSec;

    public double virtualGoalX, virtualGoalY, releaseX, releaseY;

    public ShotProjection(double[][] tofData, double[][] hoodData, double[][] rpmData, double[][] passRPMData,
            double[][] passHoodData) {
        tofLut = new Lut(tofData);
        hoodLut = new Lut(hoodData);
        rpmLut = new Lut(rpmData);
        passRPMLut = new Lut(passRPMData);
        passHoodLut = new Lut(passHoodData);

        m_distancePublisher = NetworkTableInstance.getDefault().getTable("ShotProjection").getDoubleTopic("Distance").publish();
    }

    public void solve(double rx, double ry, double rtheta, double vx, double vy,
            double gx, double gy, double releaseDelaySec) {

        double px = rx + vx * releaseDelaySec;
        double py = ry + vy * releaseDelaySec;

        double tx = gx, ty = gy;
        double dx = tx - px, dy = ty - py;
        double d = Math.sqrt(dx * dx + dy * dy);
        double t = tofLut.get(d);

        for (int i = 0; i < kMaxIterations; i++) {
            tx = gx - vx * t;
            ty = gy - vy * t;
            dx = tx - px;
            dy = ty - py;
            double nd = Math.sqrt(dx * dx + dy * dy);
            boolean converged = Math.abs(nd - d) < kConvergedMeters;
            d = nd;
            t = tofLut.get(d);
            if (converged)
                break;
        }

        distance = d;
        tofSec = t;
        robotAngleRad = Math.atan2(dy, dx) - rtheta;
        hoodDeg = hoodLut.get(d);
        rpm = rpmLut.get(d);
        virtualGoalX = tx;
        virtualGoalY = ty;
        releaseX = px;
        releaseY = py;

        m_distancePublisher.accept(d);
    }

    public void solvePassing(double rx, double ry, double rtheta, double gx, double targetAngleRad) {
        double dist = Math.abs(rx - gx);
        robotAngleRad = targetAngleRad - rtheta;
        rpm = passRPMLut.get(dist);
        hoodDeg = passHoodLut.get(dist);
    }

    public void solveIdle(double rx, double ry, double rtheta, double gx, double hubX, double hubY) {
        double dist = Math.abs(rx - gx);
        robotAngleRad = Math.atan2(hubY - ry, hubX - rx) - rtheta;
        rpm = passRPMLut.get(dist);
        hoodDeg = passHoodLut.get(dist);
    }
}

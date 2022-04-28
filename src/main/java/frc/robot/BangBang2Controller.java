package frc.robot;

import java.time.Duration;
import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.Queue;

import edu.wpi.first.math.Pair;

public class BangBang2Controller extends BangBangController {

    protected final double spinupPower;
    protected final Duration toleranceWindow;
    protected final double toleranceThreshold;
    protected final Queue<Sample> samples = new ArrayDeque<>();

    public BangBang2Controller(double spinupPower, double maintainancePower, double tolerance, Duration toleranceWindow, double toleranceThreshold) {
        super(maintainancePower, tolerance);
        this.spinupPower = spinupPower;
        this.toleranceWindow = toleranceWindow;
        this.toleranceThreshold = toleranceThreshold;
    }

    /**
     * If current measurement is below the setpoint and outside of tolerance, return {@ spinupPower}, but if below and within tolerance, return {@link maintainancePower}.
     * */
    @Override
    public synchronized double calculate(double measurement, double setpoint) {
        this.m_measurement = measurement;
        if (this.m_setpoint != setpoint) {
            this.m_setpoint = setpoint;
            this.samples.clear();
        }
        addSample();

        if (measurement < setpoint) {
            if (Math.abs(measurement - setpoint) > this.m_tolerance) {
                return power;
            } else {
                return spinupPower;
            }
        }
        return 0;
    }

    @Override
    public synchronized boolean atSetpoint() {
        int numSamples = samples.size();
        if (numSamples == 0) {
            return false;
        }
        int samplesInTolerance = 0;
        for (Sample s : samples) {
            boolean withinTolerance = Math.abs(s.getValue() - this.m_setpoint) < this.m_tolerance;
            if (withinTolerance) {
                samplesInTolerance++;
            }
        }
        return (samplesInTolerance / numSamples) >= toleranceThreshold;
    }

    public synchronized void reset() {
        this.m_measurement = 0;
        this.m_setpoint = 0;
        this.samples.clear();
    }

    private synchronized void addSample() {
        long now = System.nanoTime();
        long lookback = now - toleranceWindow.toNanos();
        samples.add(new Sample(now, this.m_measurement));
        Iterator<Sample> it = samples.iterator();
        while (it.hasNext()) {
            Sample s = it.next();
            if (s.getTimestamp() < lookback) {
                it.remove();
            } else {
                // we should be able to break early here since the queue should be ordered by timestamp
                // break;
            }
        }
    }

    public static class Sample extends Pair<Long, Double> {
        public Sample(Long timestamp, Double value) {
            super(timestamp, value);
        }

        public long getTimestamp() {
            return getFirst();
        }

        public double getValue() {
            return getSecond();
        }
    }
}


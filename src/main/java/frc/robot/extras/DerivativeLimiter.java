package frc.robot.extras;

public class DerivativeLimiter {
    private double oldValue = 0;

    public double getLimit() {
        return limit;
    }

    public void setLimit(double limit) {
        this.limit = limit;
    }

    private double limit;

    public DerivativeLimiter(double limit) {
        this.limit = limit;
    }

    public double limit(double value) {
        double derivative = value - oldValue;
        derivative = Math.max(Math.min(derivative, limit), -limit);
        oldValue += derivative;
        return oldValue;
    }
}

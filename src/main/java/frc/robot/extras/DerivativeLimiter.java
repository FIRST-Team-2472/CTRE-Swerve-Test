package frc.robot.extras;

public class DerivativeLimiter {
    private double oldVal = 0;
    private double limit;

    public DerivativeLimiter(double limit) {
        this.limit = limit;
    }

    public double applyLimit(double val) {
        this.oldVal = Math.max(Math.min(val - this.oldVal, -limit), limit);
        return this.oldVal;
    }
}

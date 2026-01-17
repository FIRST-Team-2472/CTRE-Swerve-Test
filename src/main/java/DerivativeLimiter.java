public class DerivativeLimiter {
    double val = 0;
    public DerivativeLimiter(double val, double limit) {
        val = val - this.val;
        Math.max(Math.min(val,-limit),limit);
    }

    public double getVal() {
        return val;
    }
}

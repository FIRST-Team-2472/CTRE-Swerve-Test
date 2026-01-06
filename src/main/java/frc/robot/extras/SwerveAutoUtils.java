package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveAutoUtils {
    public static double getMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static double[] directionFromPoseAndTarget(Pose2d pose, Pose2d target) {
        double x = target.getX() - pose.getX();
        double y = target.getY() - pose.getY();

        double magnitude = getMagnitude(x, y);

        // Normalize
        x /= magnitude;
        y /= magnitude;

        return new double[]{x, y, magnitude};
    }
}

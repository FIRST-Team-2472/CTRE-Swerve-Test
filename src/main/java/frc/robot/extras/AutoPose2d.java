package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.FieldConstants.K_FIELD_LENGTH;
import static frc.robot.Constants.FieldConstants.K_FIELD_WIDTH;

/**
 * Use for points along a path for autos, as `AutoPose2d` can automatically flip for the red alliance.
 * Please do not use outside of that purpose, use a `Pose2d` for internal math
 */
public class AutoPose2d extends Pose2d {

    public AutoPose2d() {
        super();
    }

    public AutoPose2d(Pose2d pose2d) {
        super(pose2d.getTranslation(), pose2d.getRotation());
    }

    public AutoPose2d(double x, double y, Rotation2d angle) {
        super(x, y, angle);
    }

    public Pose2d toPose2d() {
        DriverStation.Alliance alliance;

        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        return alliance == DriverStation.Alliance.Red
                ? new Pose2d(K_FIELD_LENGTH - getX(), K_FIELD_WIDTH - getY(),  getRotation().rotateBy(new Rotation2d(Math.PI)))
                : new Pose2d(getX(), getY(), getRotation());
    }
}

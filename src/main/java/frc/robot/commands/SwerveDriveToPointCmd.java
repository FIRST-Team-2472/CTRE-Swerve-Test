package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.OperatorPerspective;
import static frc.robot.Constants.DriveConstants.K_AUTO_ROTATION_TOLERANCE;
import static frc.robot.Constants.DriveConstants.K_AUTO_TRANSLATION_TOLERANCE;

public class SwerveDriveToPointCmd extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle driveAndTurn = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance); // Don't automatically flip heading based on alliance

    public MotorPowerController speedPowerController;

    private final Pose2d targetPosition;
    private double speed = 0.0;
    private final Timer timer;

    public SwerveDriveToPointCmd(CommandSwerveDrivetrain drivetrain, Pose2d targetPosition) {
        this.drivetrain = drivetrain;
        this.targetPosition = targetPosition;   // targetPosition is not field pose
                                                // but needs mirroring

        timer = new Timer();

        if (RobotBase.isSimulation()) {
            speedPowerController = new MotorPowerController(0.87, 0.13, .005, 1, .2, 0, 1);
            driveAndTurn.HeadingController.setPID(5.0, 0.0, 0.0);
        }
        // TODO: Find PID values for real Robot

        driveAndTurn.TargetDirection = targetPosition.getRotation();

        addRequirements(drivetrain);
    }

    double getMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    double[] directionFromPoseAndTarget(Pose2d pose, Pose2d target) {
        double x = target.getX() - pose.getX();
        double y = target.getY() - pose.getY();

        double magnitude = getMagnitude(x, y);

        // Normalize
        x /= magnitude;
        y /= magnitude;

        return new double[]{x, y, magnitude};
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        Pose2d botPose = drivetrain.getState().Pose;
        double[] direction = directionFromPoseAndTarget(botPose, targetPosition);

        speed = Math.abs(speedPowerController.calculate(0, direction[2]));

        direction[0] *= speed;
        direction[1] *= speed;

        drivetrain.setControl(driveAndTurn
                .withVelocityX(direction[0])
                .withVelocityY(direction[1])
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
    }

    @Override
    public boolean isFinished() {

        // use this function if you override the command to finish it
        if (timer.hasElapsed(10)) {
            return true;
        }

        Pose2d botPose = drivetrain.getState().Pose;

        return getMagnitude(botPose.getX() - targetPosition.getX(), botPose.getY() - targetPosition.getY()) < K_AUTO_TRANSLATION_TOLERANCE
                && Math.abs(targetPosition.getRotation().minus(botPose.getRotation()).getDegrees()) < K_AUTO_ROTATION_TOLERANCE;
    }
}

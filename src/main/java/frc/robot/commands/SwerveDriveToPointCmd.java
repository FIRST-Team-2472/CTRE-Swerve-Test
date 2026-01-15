package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.OperatorPerspective;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.extras.SwerveAutoUtils.directionFromPoseAndTarget;
import static frc.robot.extras.SwerveAutoUtils.getMagnitude;

public class SwerveDriveToPointCmd extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle driveAndTurn = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance); // Don't automatically flip heading based on alliance

    public PIDController speedPowerController;

    private final Pose2d targetPosition;
    private final Timer timer;

    public SwerveDriveToPointCmd(CommandSwerveDrivetrain drivetrain, Pose2d targetPosition) {
        this.drivetrain = drivetrain;
        this.targetPosition = targetPosition;   // targetPosition is not field pose
                                                // but needs mirroring

        timer = new Timer();

        if (RobotBase.isSimulation()) {
            speedPowerController = new PIDController(3.0, 0.03, 0.0);
            driveAndTurn.HeadingController.setPID(4.0, 0.0, 0.0);
        } else {
            speedPowerController = new PIDController(0.2, 0.13, .005);
            driveAndTurn.HeadingController.setPID(5.0, 0.0, 0.0);
        }

        driveAndTurn.TargetDirection = targetPosition.getRotation();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        Pose2d botPose = drivetrain.getState().Pose;
        double[] direction = directionFromPoseAndTarget(botPose, targetPosition);

        double pid = Math.abs(speedPowerController.calculate(0, direction[2]));
        pid = Math.min(pid, 1.0d);
        double speed = pid * K_AUTO_SPEED;

        Logger.recordOutput("PID Output", pid);

        direction[0] *= speed;
        direction[1] *= speed;

        drivetrain.setControl(driveAndTurn
                .withVelocityX(direction[0])
                .withVelocityY(direction[1])
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {

        // use this function if you override the command to finish it
        if (timer.hasElapsed(10)) {
            return true;
        }

        Pose2d botPose = drivetrain.getState().Pose;

        double distance = getMagnitude(botPose.getX() - targetPosition.getX(), botPose.getY() - targetPosition.getY());
        double rotational_error = Math.abs(targetPosition.getRotation().minus(botPose.getRotation()).getDegrees());

        boolean isThere = distance < K_AUTO_TRANSLATION_TOLERANCE
                && rotational_error < K_AUTO_ROTATION_TOLERANCE;

        if (isThere) {
            System.out.printf("%s arrived at position (%.3fm, %.3fm, %.3f°) while being %.3fm and %.3f° off.\n", getName(), botPose.getX(), botPose.getY(), botPose.getRotation().getDegrees(), distance, rotational_error);
        }

        return isThere;
    }
}

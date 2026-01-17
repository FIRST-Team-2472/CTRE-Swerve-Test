package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotorPowerController;
import frc.robot.extras.DerivativeLimiter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.OperatorPerspective;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.extras.SwerveAutoUtils.directionFromPoseAndTarget;
import static frc.robot.extras.SwerveAutoUtils.getMagnitude;

public class SwerveDriveToPointCmd extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance); // Don't automatically flip heading based on alliance

    public PIDController speedPowerController;
    public PIDController turningPowerController;
    public DerivativeLimiter speedDerivLimiter = new DerivativeLimiter(0.2);
    public DerivativeLimiter turningDerivLimiter = new DerivativeLimiter(0.2);

    private final Pose2d targetPosition;
    private final Timer timer;

    public SwerveDriveToPointCmd(CommandSwerveDrivetrain drivetrain, Pose2d targetPosition) {
        this.drivetrain = drivetrain;
        this.targetPosition = targetPosition;   // targetPosition is not field pose
                                                // but needs mirroring

        timer = new Timer();

        if (RobotBase.isSimulation()) {
            speedPowerController = new PIDController(3.0, 0.01, 0.0);
            turningPowerController = new PIDController(1.5, 0.0, 0.0);
        } else {
            speedPowerController = new PIDController(0.65, 0.005, 0.02
            );
            turningPowerController = new PIDController(2.0, 0.0, 0.0);
        }

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

        double movementPID = Math.abs(speedPowerController.calculate(0, direction[2]));
        movementPID = Math.min(movementPID, 1.0d);
        double movementSpeed = movementPID * K_AUTO_SPEED;

        // Limit Acceleration
        movementSpeed = speedDerivLimiter.limit(movementSpeed);

        double turningPID = turningPowerController.calculate(botPose.getRotation().getRadians(), targetPosition.getRotation().getRadians());
        turningPID = Math.max(Math.min(turningPID, 1.0d), -1.0);
        double turningSpeed = turningPID * K_MAX_ANGULAR_RATE;

        // Limit Acceleration
        turningSpeed = turningDerivLimiter.limit(turningSpeed);

        Logger.recordOutput("Movement Speed", movementSpeed);
        Logger.recordOutput("Turning Speed", turningSpeed);

        direction[0] *= movementSpeed;
        direction[1] *= movementSpeed;

        drivetrain.setControl(drive
                .withVelocityX(direction[0])
                .withVelocityY(direction[1])
                .withRotationalRate(turningSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {

        // use this function if you override the command to finish it
//        if (timer.hasElapsed(20)) {
//            return true;
//        }

        Pose2d botPose = drivetrain.getState().Pose;

        double distance = getMagnitude(botPose.getX() - targetPosition.getX(), botPose.getY() - targetPosition.getY());
        double rotational_error = Math.abs(targetPosition.getRotation().minus(botPose.getRotation()).getDegrees());

        Logger.recordOutput("Movement Distance", distance);
        Logger.recordOutput("Turning Distance", rotational_error);

        boolean isThere = distance < K_AUTO_TRANSLATION_TOLERANCE
                && rotational_error < K_AUTO_ROTATION_TOLERANCE;

        if (isThere) {
            System.out.printf("%s arrived at position (%.3fm, %.3fm, %.3f°) while being %.3fm and %.3f° off.\n", getName(), botPose.getX(), botPose.getY(), botPose.getRotation().getDegrees(), distance, rotational_error);
        }

        return isThere;
    }
}

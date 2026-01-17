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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.extras.SwerveAutoUtils.directionFromPoseAndTarget;
import static frc.robot.extras.SwerveAutoUtils.getMagnitude;

public class SwerveFollowTransitionCmd extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance); // Don't automatically flip heading based on alliance

    public PIDController speedPowerController;
    public PIDController turningPowerController;

    Pose2d startPose, endPose, targetPose;
    Pose2d drivePose;
    double xTransitionPerFrame, yTransitionPerFrame, angleTransitionPerFrame;

    Timer timer = new Timer();

    /**
     * @param drivetrain     the swerve subsystem
     * @param startPose      PosPose2d of the startPose
     * @param endPose        the pose to transition to and end at
     * @param transitionTime the time it should take to fully transition to the end pose
     */
    public SwerveFollowTransitionCmd(CommandSwerveDrivetrain drivetrain, Pose2d startPose, Pose2d endPose, double transitionTime) {

        this.drivetrain = drivetrain;
        this.startPose = startPose;
        this.endPose = endPose;
        targetPose = this.startPose;

        if (RobotBase.isSimulation()) {
            speedPowerController = new PIDController(3.0, 0.01, 0.0);
            turningPowerController = new PIDController(1.5, 0.0, 0.0);
        } else {
            speedPowerController = new PIDController(0.65, 0.005, 0.02
            );
            turningPowerController = new PIDController(2.0, 0.0, 0.0);
        }

        xTransitionPerFrame = (endPose.getX() - startPose.getX()) / 50 / transitionTime;// 50 is code refreshes per second
        yTransitionPerFrame = (endPose.getY() - startPose.getY()) / 50 / transitionTime;
        angleTransitionPerFrame = endPose.getRotation().minus(startPose.getRotation()).getDegrees() / 50 / transitionTime;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        calculateCurrentPose();

        Pose2d botPose = drivetrain.getState().Pose;
        double[] direction = directionFromPoseAndTarget(botPose, drivePose);

        double movementPID = Math.abs(speedPowerController.calculate(0, direction[2]));
        movementPID = Math.min(movementPID, 1.0d);
        double movementSpeed = movementPID * K_AUTO_SPEED;

        double turningPID = turningPowerController.calculate(botPose.getRotation().getRadians(), drivePose.getRotation().getRadians());
        turningPID = Math.max(Math.min(turningPID, 1.0d), -1.0);
        double turningSpeed = turningPID * K_MAX_ANGULAR_RATE;

        Logger.recordOutput("Movement PID Output", movementPID);
        Logger.recordOutput("Turning PID Output", turningPID);

        direction[0] *= movementSpeed;
        direction[1] *= movementSpeed;

        drivetrain.setControl(drive
                .withVelocityX(direction[0])
                .withVelocityY(direction[1])
                .withRotationalRate(turningSpeed)
        );
    }

    public void calculateCurrentPose() {
        //if we are at the end pose we will just return
        //the < .01 is because doubles rarely exactly equal eachother
        if (Math.abs(targetPose.getX() - endPose.getX()) < .01 && Math.abs(targetPose.getY() - endPose.getY()) < .01 && Math.abs(targetPose.getRotation().minus(endPose.getRotation()).getDegrees()) < .01) {
            targetPose = endPose;
            return;
        }

        //creating a new pose by adding the transition per frame to the old one
        targetPose = new Pose2d(targetPose.getX() + xTransitionPerFrame, targetPose.getY() + yTransitionPerFrame, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees() + angleTransitionPerFrame));
        drivePose = targetPose;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {

        // use this function if you override the command to finish it
        if (timer.hasElapsed(20)) {
            return true;
        }

        Pose2d botPose = drivetrain.getState().Pose;

        double distance = getMagnitude(botPose.getX() - endPose.getX(), botPose.getY() - endPose.getY());
        double rotational_error = Math.abs(endPose.getRotation().minus(botPose.getRotation()).getDegrees());

        boolean isThere = distance < K_AUTO_TRANSLATION_TOLERANCE
                && rotational_error < K_AUTO_ROTATION_TOLERANCE;

        if (isThere) {
            System.out.printf("%s arrived at position (%.3fm, %.3fm, %.3f°) while being %.3fm and %.3f° off.\n", getName(), botPose.getX(), botPose.getY(), botPose.getRotation().getDegrees(), distance, rotational_error);
        }

        return isThere;
    }
}

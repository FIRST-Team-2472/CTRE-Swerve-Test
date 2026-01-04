package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;


public class TeleOpDriveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private Supplier<Double> inputX, inputY, inputTheta;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.K_MAX_SPEED * 0.1).withRotationalDeadband(DriveConstants.K_MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public TeleOpDriveCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> inputX, Supplier<Double> inputY, Supplier<Double> inputTheta) {
        this.drivetrain = drivetrain;
        this.inputX = inputX;
        this.inputY = inputY;
        this.inputTheta = inputTheta;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrain);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()} returns true.)
     */
    @Override
    public void execute() {
        drivetrain.setControl(
                drive.withVelocityX(this.inputY.get() * DriveConstants.K_MAX_SPEED)
                        .withVelocityY(this.inputX.get() * DriveConstants.K_MAX_SPEED)
                        .withRotationalRate(this.inputTheta.get() * DriveConstants.K_MAX_ANGULAR_RATE)
        );
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     *
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is, it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}

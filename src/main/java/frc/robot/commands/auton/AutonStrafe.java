package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveDrivebase;

/**
 * Runs the drivebase autonomously in strafe mode.
 * 
 * @author Benjamin Hall
 */
public class AutonStrafe extends CommandBase {
    private final SwerveDrivebase kDrivebase;
    private final double kDistance;
    private final double kDirection;

    /**
     * Runs the drivebase autonomously in strafe mode.
     * 
     * @param drivebase
     *        The drivebase to run
     * @param distance
     *        The distance to travel, in encoder ticks
     * @param direction
     *        The direction to travel, in degrees, relative to the field (pigeon angle of 0)
     */
    public AutonStrafe(SwerveDrivebase drivebase, double distance, double direction) {
        addRequirements(drivebase);
        this.kDrivebase = drivebase;
        this.kDistance = distance;
        this.kDirection = direction;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        kDrivebase.resetDriveEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        kDrivebase.autonDrive(kDistance, kDirection);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        kDrivebase.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return kDrivebase.hasReachedTarget(kDistance, kDirection, 1000);
    }
}

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team217.Num;

import frc.robot.subsystems.SwerveDrivebase;

/**
 * Runs the drivebase in teleop control mode.
 * 
 * @author Benjamin Hall
 */
public class TeleopDrive extends CommandBase {
    private final SwerveDrivebase kDrivebase;
    private final Joystick kDriver;

    /**
     * Runs the drivebase in teleop control mode.
     * 
     * @param drivebase
     *        The drivebase to run
     * @param driver
     *        The driver joystick
     */
    public TeleopDrive(SwerveDrivebase drivebase, Joystick driver) {
        addRequirements(drivebase);
        this.kDrivebase = drivebase;
        this.kDriver = driver;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = Num.deadband(-kDriver.getY(), 0.1);
        double strafe = Num.deadband(kDriver.getX(), 0.1);
        double turn = Num.deadband(kDriver.getZ(), 0.1);

        kDrivebase.set(speed, strafe, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        kDrivebase.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

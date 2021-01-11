package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team217.Num;

import frc.robot.subsystems.SwerveDrivebase;

public class TeleopDrive extends CommandBase {
    private SwerveDrivebase drivebase;
    private Joystick driver;

    /**
     * Runs the drivebase in teleop control mode.
     * 
     * @param drivebase
     *        The drivebase to run
     */
    public TeleopDrive(SwerveDrivebase drivebase, Joystick driver) {
        addRequirements(drivebase);
        this.drivebase = drivebase;
        this.driver = driver;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = Num.deadband(-driver.getY(), 0.1);
        double strafe = Num.deadband(driver.getX(), 0.1);
        double turn = Num.deadband(driver.getZ(), 0.1);

        drivebase.set(speed, strafe, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

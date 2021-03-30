package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.teleop.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    // Joysticks
    public final Joystick kDriver = new Joystick(Constants.kDriverPort);

    // Subsystems
    public final SwerveDrivebase kSwerveDrivebase = new SwerveDrivebase(28.0, 22.0);

    public Command getTeleopCommand() {
        return new TeleopDrive(kSwerveDrivebase, kDriver);
    }

    public void simulationInit() {
        kSwerveDrivebase.simulationInit();
    }
}

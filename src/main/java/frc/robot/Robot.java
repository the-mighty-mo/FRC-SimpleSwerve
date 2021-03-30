// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer kRobotContainer = new RobotContainer();

    private Command teleopCommand;

    @Override
    public void simulationInit() {
        kRobotContainer.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        kRobotContainer.kSwerveDrivebase.initTurnEncoders();
        kRobotContainer.kSwerveDrivebase.resetPigeon();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        teleopCommand = kRobotContainer.getTeleopCommand();

        if (teleopCommand != null) {
            teleopCommand.schedule();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}

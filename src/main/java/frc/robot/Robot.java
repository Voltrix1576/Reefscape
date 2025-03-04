// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.SwerveDriveCommand;
import frc.robot.Subsystems.Limelight.limelight;
import frc.robot.Subsystems.Swerve.SwerveDriveTrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private limelight aprilTagLimelight = new limelight("limelight");

  public Robot() {
    m_robotContainer = new RobotContainer();
    SwerveDriveTrain.getInstance();
    SwerveDriveTrain.getInstance().resetEncoders();
    CameraServer.startAutomaticCapture("Elevator Camera", 1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    aprilTagLimelight.periodic();

    SmartDashboard.putNumber("AprilTagLimelightX", aprilTagLimelight.getX());
    SmartDashboard.putNumber("AprilTagLimelightY", aprilTagLimelight.getY());
    SmartDashboard.putNumber("AprilTagLimelightArea", aprilTagLimelight.getArea());
    SmartDashboard.putNumber("AprilTagLimelightID", aprilTagLimelight.getTagId());
    
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SwerveDriveTrain.getInstance().resetEncoders();

    CommandScheduler.getInstance().setDefaultCommand(SwerveDriveTrain.getInstance(), new SwerveDriveCommand());

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

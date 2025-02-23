// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Swerve.SwerveDriveTrain;

public class RobotContainer {
  public static CommandXboxController driverController = new CommandXboxController(0);
  private double startTime;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverController.y().whileTrue(new InstantCommand(() -> SwerveDriveTrain.getInstance().updateOffset()));

    // driverController.leftTrigger().whileTrue(
    //  new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.2)))
    // .whileFalse(new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(1)));

    driverController.leftTrigger().whileTrue(
     new InstantCommand(() -> ArmSubsystem.getInstance().setArmVol(5)))
    .whileFalse(new InstantCommand(() -> ArmSubsystem.getInstance().setArmVol(0)));

    driverController.rightTrigger().onTrue(
      new SequentialCommandGroup(
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setSliderMotorPower(0.4)),
      new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
      new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1.5),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setSliderMotorPower(0))));

     driverController.rightBumper().onTrue( //L4
      new SequentialCommandGroup(
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(-2)),
        new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.15)),
        new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(90))));


     driverController.rightStick().onTrue(  //L3
      new SequentialCommandGroup(
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(-2.6)),
        new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.25)),
        new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(43))));

      driverController.leftStick().onTrue( //L2
      new SequentialCommandGroup(
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(-2.6)),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(19))));

      driverController.leftBumper().onTrue(
      new SequentialCommandGroup(  //L1 / 0
      new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(1)),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(7)),
      new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
      new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1.5),
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(1.3))));

      driverController.povRight().onTrue(
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(-2.3)));

        driverController.povLeft().onTrue(
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(-7)));

        driverController.leftTrigger().onTrue(
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(-4)));
      
       driverController.x().whileTrue(
        new InstantCommand(() -> ArmSubsystem.getInstance().setIntakePower(1)))
        .whileFalse(new InstantCommand(() -> ArmSubsystem.getInstance().setIntakePower(0)));

      driverController.b().whileTrue(
        new InstantCommand(() -> ArmSubsystem.getInstance().setIntakePower(-1)))
        .whileFalse(new InstantCommand(() -> ArmSubsystem.getInstance().setIntakePower(0)));


      driverController.povUp().whileTrue(
        new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPower(0.2)))
        .whileFalse(new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPower(0)));
       
       
       driverController.povDown().whileTrue(
        new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPower(-0.2)))
        .whileFalse(new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPower(0)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  } 
}

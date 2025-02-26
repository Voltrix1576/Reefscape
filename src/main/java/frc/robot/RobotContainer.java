// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Swerve.SwerveDriveTrain;

public class RobotContainer {
  public static CommandXboxController driverController = new CommandXboxController(0);
  public static CommandPS4Controller operatorController = new CommandPS4Controller(1);
  private final SendableChooser<Command> autoChooser;
  private double startTime;

  public RobotContainer() {
    NamedCommands.registerCommand("ScoreL4", ScoreL4());
    NamedCommands.registerCommand("ResetElevator", ResetElevator());
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Auto Up", new PathPlannerAuto("Auto1Up"));
    autoChooser.addOption("Auto Middle", new PathPlannerAuto("Auto2Middle"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    driverController.y().whileTrue(new InstantCommand(() -> SwerveDriveTrain.getInstance().updateOffset()));

    driverController.leftTrigger().whileTrue(
     new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.2)))
    .whileFalse(new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(1)));

    driverController.rightTrigger().onTrue(
      new SequentialCommandGroup(
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setSliderMotorPower(0.4)),
      new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
      new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1.5),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setSliderMotorPower(0))));

     driverController.rightBumper().onTrue( //L4
      new SequentialCommandGroup(
        new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.15)),
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(2)),
        new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(82.4))));


     driverController.rightStick().onTrue(  //L3
      new SequentialCommandGroup(
        new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.3)),
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(2.6)),
        new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(40))));

      driverController.leftStick().onTrue( //L2
      new SequentialCommandGroup(
        new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(0.5)),
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(2.6)),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(14.4))));

      driverController.leftBumper().onTrue(
      new SequentialCommandGroup(  //L1 / 0
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(2)),
      new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
      new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1.5),
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(0.3)),
      new InstantCommand(() -> SwerveDriveTrain.getInstance().setVelocityFactor(1))));

      operatorController.L2().onTrue( // Ground intake
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(2.7)));

        operatorController.povDown().onTrue( // L2 intake
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(3.7)));        

      operatorController.R1().onTrue( // Net Pos
        new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(7.3)));

      operatorController.L1().onTrue( // Proccesor
          new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(0.7)));

      operatorController.povUp().onTrue( // L3 intake
          new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(4)));

      operatorController.triangle().onTrue(
        new InstantCommand(() -> ArmSubsystem.getInstance().resetArm()));
      
      operatorController.R2().whileTrue( //intake
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

  private Command ScoreL4() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(2)),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(82.4)),
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setSliderMotorPower(0.4)));
  }

  private Command ResetElevator() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPose(2)),
      new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
      new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1),
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmPose(0.3)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  } 
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;
  private TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
  private TalonFX intakeMotor = new TalonFX(ArmConstants.INTAKE_MOTOR_ID);
  private MotionMagicVoltage armMotionMagicVoltage = new MotionMagicVoltage(0);
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private StatusSignal<Angle> armPose = armMotor.getPosition();
  
   public ArmSubsystem() {
    configArmMotor();
    configIntakeMotor();
   }

   private void configIntakeMotor() {
    TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();

    intakeMotorConfiguration.MotorOutput.Inverted = ArmConstants.IS_INTAKE_MOTOR_INVERTED ?
    InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    intakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
   }

  private void configArmMotor() {
    TalonFXConfiguration armMotorConfiguration = new TalonFXConfiguration();
    MotionMagicConfigs armMotionMagicConfigs = new MotionMagicConfigs();

    armMotorConfiguration.MotorOutput.Inverted = ArmConstants.IS_ARM_MOTOR_INVERTED ? 
    InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    armMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

     armMotorConfiguration.Slot0.kP = ArmConstants.ARM_P;
     armMotorConfiguration.Slot0.kI = ArmConstants.ARM_I;
     armMotorConfiguration.Slot0.kD = ArmConstants.ARM_D;

    armMotionMagicConfigs.MotionMagicCruiseVelocity = 90;
    armMotionMagicConfigs.MotionMagicAcceleration = 100;  

     armMotor.getConfigurator().apply(armMotorConfiguration);
  }

  public void setArmPower(double power) {
    armMotor.set(power);
  }

  public void setArmPose(double setPoint) {
    armMotor.setControl(armMotionMagicVoltage.withPosition(setPoint));
    //armMotor.setControl(positionVoltage.withPosition(setPoint));
  }

  public double getArmPose() {
    armPose.refresh();
    return armPose.getValueAsDouble();
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Pose", getArmPose());
    System.out.println(armMotor.get());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;
  private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private StatusSignal<Angle> elevatorPose = elevatorMotor.getPosition();
  private SparkFlex sliderMotor = new SparkFlex(ElevatorConstants.SLIDER_MOTOR_ID , MotorType.kBrushless);
  private DigitalInput elevatorLimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);

  public ElevatorSubsystem() {
    configElevatorMotor();
    configSliderMotor();
    elevatorMotor.setPosition(0);
  }

  private void configElevatorMotor() {
    TalonFXConfiguration elevatorMotorConfiguration = new TalonFXConfiguration();
    MotionMagicConfigs elevatorMotionMagicConfigs = elevatorMotorConfiguration.MotionMagic;
    elevatorMotorConfiguration.MotorOutput.Inverted = ElevatorConstants.IS_ELEVATOR_MOTOR_INVERTED
     ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
     elevatorMotorConfiguration .MotorOutput.NeutralMode = NeutralModeValue.Brake;
     
     elevatorMotorConfiguration.Slot0.kP = ElevatorConstants.ELEVATOR_P;
     elevatorMotorConfiguration.Slot0.kI = ElevatorConstants.ELEVATOR_I;
     elevatorMotorConfiguration.Slot0.kD = ElevatorConstants.ELEVATOR_D;

     elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = 110;
     elevatorMotionMagicConfigs.MotionMagicAcceleration = 140;

     elevatorMotor.getConfigurator().apply(elevatorMotorConfiguration);
  }

  private void configSliderMotor() {
    SparkFlexConfig slideMotorConfig = new SparkFlexConfig();

    slideMotorConfig.inverted(ElevatorConstants.IS_SLIDER_MOTOR_INVERTED);
    slideMotorConfig.idleMode(IdleMode.kBrake);

    slideMotorConfig.apply(slideMotorConfig);
  }

  public void setSliderMotorPower(double power){
    sliderMotor.set(power);
  }

  public void setElevatorPose(double setPoint) {
      elevatorMotor.setControl(motionMagicVoltage.withPosition(setPoint));
  }

  public void setElevatorPower(double power) {
    elevatorMotor.set(power);
  }

  public double getElevatorPose() {
    elevatorPose.refresh();
    return elevatorPose.getValueAsDouble();
  }

  public void resetElevator() {
    if (elevatorLimitSwitch.get()) {
      if (getElevatorPose() != 0) {
        elevatorMotor.setPosition(0);  
      }
       
    }
  }

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Pose", getElevatorPose());
    SmartDashboard.putBoolean("Limit Swith", elevatorLimitSwitch.get());
    resetElevator();
  }
}

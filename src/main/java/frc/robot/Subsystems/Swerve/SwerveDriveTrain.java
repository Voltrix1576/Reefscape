// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
  private static SwerveDriveTrain instance;
  
  public double maxV = SwerveConstants.MAX_VELOCITY;
  public double maxAV = SwerveConstants.MAX_ANGULAR_VELOCITY;

  public double offsetAngle = 90;

  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

  private SwerveModule frontLeftModule = new SwerveModule(
    SwerveConstants.FRONT_LEFT_DRIVE_ID,
    SwerveConstants.FRONT_LEFT_TURNING_ID,
    SwerveConstants.FRONT_LEFT_ABSULOTE_ENCODER_ID,
    SwerveConstants.FRONT_LEFT_ABSULOTE_ENCODER_OFFSET,
    SwerveConstants.FRONT_LEFT_IS_DRIVE_MOTOR_INVERTED,
    SwerveConstants.FRONT_LEFT_IS_TURNING_MOTOR_INVERTED,
    SwerveConstants.FRONT_LEFT_IS_ABSULOTE_ENCODER_INVERTED);

  private SwerveModule frontRightModule = new SwerveModule(
    SwerveConstants.FRONT_RIGHT_DRIVE_ID,
    SwerveConstants.FRONT_RIGHT_TURNING_ID,
    SwerveConstants.FRONT_RIGHT_ABSULOTE_ENCODER_ID,
    SwerveConstants.FRONT_RIGHT_ABSULOTE_ENCODER_OFFSET,
    SwerveConstants.FRONT_RIGHT_IS_DRIVE_MOTOR_INVERTED,
    SwerveConstants.FRONT_RIGHT_IS_TURNING_MOTOR_INVERTED,
    SwerveConstants.FRONT_RIGHT_IS_ABSULOTE_ENCODER_INVERTED);

  private SwerveModule rearLeftModule = new SwerveModule(
    SwerveConstants.REAR_LEFT_DRIVE_ID,
    SwerveConstants.REAR_LEFT_TURNING_ID,
    SwerveConstants.REAR_LEFT_ABSULOTE_ENCODER_ID,
    SwerveConstants.REAR_LEFT_ABSULOTE_ENCODER_OFFSET,
    SwerveConstants.REAR_LEFT_IS_DRIVE_MOTOR_INVERTED,
    SwerveConstants.REAR_LEFT_IS_TURNING_MOTOR_INVERTED,
    SwerveConstants.REAR_LEFT_IS_ABSULOTE_ENCODER_INVERTED);

  private SwerveModule rearRightModule = new SwerveModule(
    SwerveConstants.REAR_RIGHT_DRIVE_ID,
    SwerveConstants.REAR_RIGHT_TURNING_ID,
    SwerveConstants.REAR_RIGHT_ABSULOTE_ENCODER_ID,
    SwerveConstants.REAR_RIGHT_ABSULOTE_ENCODER_OFFSET,
    SwerveConstants.REAR_RIGHT_IS_DRIVE_MOTOR_INVERTED,
    SwerveConstants.REAR_RIGHT_IS_TURNING_MOTOR_INVERTED,
    SwerveConstants.REAR_RIGHT_IS_ABSULOTE_ENCODER_INVERTED);

  private final Translation2d frontLeftLocation = new Translation2d(SwerveConstants.WIDTH / 2, SwerveConstants.LENGTH / 2);
  private final Translation2d frontRightLocation = new Translation2d(SwerveConstants.WIDTH / 2, -(SwerveConstants.LENGTH / 2));
  private final Translation2d rearLeftLocation = new Translation2d(-(SwerveConstants.WIDTH / 2), SwerveConstants.LENGTH / 2);
  private final Translation2d rearRightLocation = new Translation2d(-(SwerveConstants.WIDTH / 2), -(SwerveConstants.LENGTH / 2));

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, 
  frontRightLocation,
  rearLeftLocation, rearRightLocation); 

  // new Translation2d(SwerveConstants.WIDTH / 2.0, SwerveConstants.WIDTH / 2.0), //front left
  // new Translation2d(SwerveConstants.WIDTH / 2.0, -SwerveConstants.WIDTH / 2.0), // front right
  // new Translation2d(-SwerveConstants.WIDTH / 2.0, SwerveConstants.WIDTH / 2.0), //rear left
  // new Translation2d(-SwerveConstants.WIDTH / 2.0, -SwerveConstants.WIDTH / 2.0) //rear right

  public SwerveDriveTrain() {
    
    navx.reset();
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearLeftModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  public void setVelocityFactor(double factor) {
    maxV = SwerveConstants.MAX_VELOCITY * factor;
    maxAV = SwerveConstants.MAX_ANGULAR_VELOCITY * factor;
  }

  public double getAngle() {
    return navx.getYaw();
  }

  public void updateOffset() {
    offsetAngle = getAngle();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getAngle()));
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void setSwerveState(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY);
    frontLeftModule.setDesiredState(states[2]);
    frontRightModule.setDesiredState(states[3]);
    rearLeftModule.setDesiredState(states[0]);
    rearRightModule.setDesiredState(states[1]);
  }

  public void drive(double xv, double yv, double omega, boolean isFieldrelative) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
      isFieldrelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xv, yv, omega,
      new Rotation2d(Math.toRadians(getAngle() - offsetAngle)))
      : new ChassisSpeeds(xv, yv, omega)
    );
    setSwerveState(states);
  }

  public static SwerveDriveTrain getInstance() {
    if (instance == null) {
      instance = new SwerveDriveTrain();
    }
    return instance;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left abs", frontLeftModule.getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Front Right abs", frontRightModule.getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Rear Left abs", rearLeftModule.getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Rear Right abs", rearRightModule.getAbsoluteEncoderPosition());

    SmartDashboard.putNumber("Front Left Angle", frontLeftModule.getTurningPose());
    SmartDashboard.putNumber("Front Right Angle", frontRightModule.getTurningPose());
    SmartDashboard.putNumber("Rear Left Angle", rearLeftModule.getTurningPose());
    SmartDashboard.putNumber("Rear Right Angle", rearRightModule.getTurningPose());

    SmartDashboard.putNumber("navx", getAngle());

    //rearRightModule.driveMotorSetPower(0.4);
    //rearRightModule.turningMotorSetPower(0.4);
    

  }
}

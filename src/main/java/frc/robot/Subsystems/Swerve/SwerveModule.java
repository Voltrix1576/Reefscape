package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModule {
    private TalonFX driveMotor;
    private SparkMax turningMotor;
    
    private CANcoder absoluteEncoder;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV);

    private boolean isAbsoluteEncoderInverted;
    private double offsetEncoder;

    private boolean isDriveMotorInverted;
    private boolean isTurningMotorInverted;

    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Angle> absAngle;

    private RelativeEncoder turningEncoder;

    private VelocityVoltage velocitySetter = new VelocityVoltage(0);
    private SparkClosedLoopController turningPIDController;

    public SwerveModule(int driveMotorId, int turningMotorId, int absoluteEncoderId, double offsetEncoder,
    boolean isDriveMotorInverted, boolean isTurningMotorInverted, boolean isAbsoluteEncoderInverted) {
        
        this.driveMotor = new TalonFX(driveMotorId);
        this.turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(absoluteEncoderId);

        this.isDriveMotorInverted = isDriveMotorInverted;
        this.isTurningMotorInverted = isTurningMotorInverted;
        this.isAbsoluteEncoderInverted = isAbsoluteEncoderInverted;

        this.offsetEncoder = offsetEncoder;
        this.drivePosition = driveMotor.getPosition();
        this.driveVelocity = driveMotor.getVelocity();
        this.absAngle = absoluteEncoder.getAbsolutePosition();
        this.turningEncoder = turningMotor.getEncoder();
        this.turningPIDController = turningMotor.getClosedLoopController();

        configTurningMotor();
        configDriveMotor();
        resetEncoders();
    }

    private void configTurningMotor() {
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        turningConfig.inverted(isTurningMotorInverted);
        turningConfig.idleMode(IdleMode.kBrake);
        turningConfig.encoder.positionConversionFactor(SwerveConstants.POSE_TURNING_FACTOR);
        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        turningConfig.closedLoop.pid(SwerveConstants.TURNING_KP, SwerveConstants.TURNING_KI, SwerveConstants.TURNING_KD);

        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configDriveMotor() {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        
        driveConfiguration.MotorOutput.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive 
        : InvertedValue.CounterClockwise_Positive;

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfiguration.Slot0.kP = SwerveConstants.DRIVE_KP;
        driveConfiguration.Slot0.kI = SwerveConstants.DRIVE_KI;
        driveConfiguration.Slot0.kD = SwerveConstants.DRIVE_KD;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderPosition() - offsetEncoder);
    }

    public double getAbsoluteEncoderPosition() {
        absAngle.refresh();
        return isAbsoluteEncoderInverted ? 360 - (absAngle.getValueAsDouble() + 0.5) * 360 : (absAngle.getValueAsDouble() + 0.5) * 360;
    }

    public double getTurningPose() {
        return turningEncoder.getPosition();
    }

    public double getDrivePose() {
        drivePosition.refresh();
        return drivePosition.getValueAsDouble() * SwerveConstants.DISTANCE_PER_PULSE;
    }

    public double getDriveVelocity() {
        driveVelocity.refresh();
        return driveVelocity.getValueAsDouble() * SwerveConstants.DISTANCE_PER_PULSE;
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    public void stop() {
        turningMotorSetPower(0);
        driveMotorSetPower(0);
    }

    public void turningUsingPID(double setPoint) {
        turningPIDController.setReference(setPoint, ControlType.kPosition);
    }

    public void driveUsingPID(double setPoint) {
        driveMotor.setControl(velocitySetter.withFeedForward(feedforward.calculate(setPoint)).withSlot(0));
    }

    public SwerveModulePosition getModulePose() {
        return new SwerveModulePosition(getDrivePose(), Rotation2d.fromDegrees(getTurningPose()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPose()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModule.optimize(desiredState, getTurningPose());

        if (optimizedState.speedMetersPerSecond != 0) {
            turningUsingPID(optimizedState.angle.getDegrees());
        } else {
            turningMotorSetPower(0);
        }
        driveUsingPID(optimizedState.speedMetersPerSecond);
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;
        double targetAngle = currentAngle + angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        if (angleDiff <= -270) {
            targetAngle += 360;;
        } else if (-90 > angleDiff && angleDiff > -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        } else if (90 < angleDiff && angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        } else if (angleDiff >=270) {
            targetAngle -= 360;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));

    }

}

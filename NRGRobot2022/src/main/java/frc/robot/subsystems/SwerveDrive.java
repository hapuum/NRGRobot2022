// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {


  /*Swerve Module helper class*/
  private class Module {
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 2048;
    private static final double kDrivePulsesPerMeter = kEncoderResolution / (2*kWheelRadius*Math.PI); // pulses per meter


    private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;

    // private final Encoder m_driveEncoder;
    private final CANCoder m_turningEncoder;


    /* TODO: Tune PID for drive and turning PID controllers */
    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
        1,
        0,
        0,
        new TrapezoidProfile.Constraints(
            kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param turningEncodeChannel        
     */
    public Module(
        int driveMotorChannel,
        int turningMotorChannel,
        int turningEncodeChannel


        ) {
      m_driveMotor = new TalonFX(driveMotorChannel);
      m_turningMotor = new TalonFX(turningMotorChannel);

      m_turningEncoder = new CANCoder(turningEncodeChannel);
        
      m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

      // Limit the PID Controller's input range between -pi and pi and set the input
      // to be continuous.
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveMotorVelocity(), Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
    }

    public double getDriveMotorPosition(){
      return m_driveMotor.getSelectedSensorPosition() / kDrivePulsesPerMeter;
    }

    public double getDriveMotorVelocity(){ 
      return m_driveMotor.getSelectedSensorVelocity() / kDrivePulsesPerMeter;

    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      
      // Optimize the reference state to avoid spinning further than 90 degrees
      Rotation2d currentAngle = Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

      // Calculate the drive output from the drive PID controller.
      final double driveOutput = m_drivePIDController.calculate(getDriveMotorVelocity(), state.speedMetersPerSecond);

      final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(currentAngle.getRadians(), state.angle.getRadians());

      final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

      final double batteryVolatage = RobotController.getBatteryVoltage();

      m_driveMotor.set(ControlMode.PercentOutput, (driveOutput + driveFeedforward) / batteryVolatage);
      m_turningMotor.set(ControlMode.PercentOutput, (turnOutput + turnFeedforward) / batteryVolatage);
    }
  }

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final Module m_frontLeft = new Module(1, 2, 3);
  private final Module m_frontRight = new Module(4, 5, 6);
  private final Module m_backLeft = new Module(7, 8, 9);
  private final Module m_backRight = new Module(10, 11, 12);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public SwerveDrive() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

}

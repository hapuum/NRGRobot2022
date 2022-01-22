// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {


  /*Swerve Module helper class*/
  private class Module {
    private static final double kWheelRadius = 0.047625; //1.875 inches in meters
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final MotorController m_driveMotor;
    private final MotorController m_turningMotor;

    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;


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
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public Module(
        int driveMotorChannel,
        int turningMotorChannel,
        int driveEncoderChannelA,
        int driveEncoderChannelB,
        int turningEncoderChannelA,
        int turningEncoderChannelB) {
      m_driveMotor = new PWMSparkMax(driveMotorChannel);
      m_turningMotor = new PWMSparkMax(turningMotorChannel);

      m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
      m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

      // Set the distance per pulse for the drive encoder. We can simply use the
      // distance traveled for one rotation of the wheel divided by the encoder
      // resolution.
      m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

      // Set the distance (in this case, angle) per pulse for the turning encoder.
      // This is the the angle through an entire rotation (2 * pi) divided by the
      // encoder resolution.
      m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

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
      return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

      // Calculate the drive output from the drive PID controller.
      final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

      final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

      final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

      m_driveMotor.setVoltage(driveOutput + driveFeedforward);
      m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
  }

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  //13.75 and 9.75 inches
  private final Translation2d m_frontLeftLocation = new Translation2d(0.34925, 0.24765);
  private final Translation2d m_frontRightLocation = new Translation2d(0.34925, -0.24765);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.34925, 0.24765);
  private final Translation2d m_backRightLocation = new Translation2d(-0.34925, -0.24765);

  private final Module m_frontLeft = new Module(1, 2, 0, 1, 2, 3);
  private final Module m_frontRight = new Module(3, 4, 4, 5, 6, 7);
  private final Module m_backLeft = new Module(5, 6, 8, 9, 10, 11);
  private final Module m_backRight = new Module(7, 8, 12, 13, 14, 15);

  private final AHRS m_ahrs = new AHRS(SerialPort.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d());

  public SwerveDrive() {
    m_ahrs.reset();
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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
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
        getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  /** Returns the current orientation of the robot as a Rotation2d object */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_ahrs.getAngle());
  }

}

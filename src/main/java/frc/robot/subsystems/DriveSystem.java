// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveSystem extends SubsystemBase {

  // drive
  private final CANSparkMax frontLeftDrive;
  private final CANSparkMax frontRightDrive;
  private final CANSparkMax backLeftDrive;
  private final CANSparkMax backRightDrive;

  // rotate
  private final CANSparkMax frontLeftRotate;
  private final CANSparkMax frontRightRotate;
  private final CANSparkMax backLeftRotate;
  private final CANSparkMax backRightRotate;

  // module
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  // gyro
  private final AHRS gyro;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private boolean fieldOriented;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    // drive
    frontLeftDrive = new CANSparkMax(FRONT_LEFT_DRIVE, MotorType.kBrushless);
    frontRightDrive = new CANSparkMax(FRONT_RIGHT_DRIVE, MotorType.kBrushless);
    backLeftDrive = new CANSparkMax(BACK_LEFT_DRIVE, MotorType.kBrushless);
    backRightDrive = new CANSparkMax(BACK_RIGHT_DRIVE, MotorType.kBrushless);

    // rotate
    frontLeftRotate = new CANSparkMax(FRONT_LEFT_ROTATE, MotorType.kBrushless);
    frontRightRotate = new CANSparkMax(FRONT_RIGHT_ROTATE, MotorType.kBrushless);
    backLeftRotate = new CANSparkMax(BACK_LEFT_ROTATE, MotorType.kBrushless);
    backRightRotate = new CANSparkMax(BACK_RIGHT_ROTATE, MotorType.kBrushless);

    // modules
    frontLeft = new SwerveModule(frontLeftDrive, frontLeftRotate);
    frontRight = new SwerveModule(frontRightDrive, frontRightRotate);
    backLeft = new SwerveModule(backLeftDrive, backLeftRotate);
    backRight = new SwerveModule(backRightDrive, backRightRotate);
    
    // gyro
    gyro = new AHRS();

    kinematics = new SwerveDriveKinematics(
      new Translation2d(MODULE_DIST, MODULE_DIST), // front left
      new Translation2d(MODULE_DIST, -MODULE_DIST), // front right
      new Translation2d(-MODULE_DIST, MODULE_DIST), // back left
      new Translation2d(-MODULE_DIST, -MODULE_DIST) // back right
    );

    odometry = new SwerveDriveOdometry(
      kinematics, 
      Rotation2d.fromDegrees(gyro.getAngle())
    );
  }

  /**
   * drive the robot
   * 
   * @param x m/s
   * @param y m/s
   * @param theta rad/s
   */
  public void drive(double x, double y, double theta) {
    ChassisSpeeds speeds;
    
    // field oriented vs robot oriented
    if (fieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        MathUtil.clamp(x, -MAX_X_SPEED, MAX_X_SPEED), 
        MathUtil.clamp(y, -MAX_Y_SPEED, MAX_Y_SPEED), 
        MathUtil.clamp(theta, -MAX_ROT_SPEED, MAX_ROT_SPEED), 
        Rotation2d.fromDegrees(gyro.getAngle())
      );
    } else {
      speeds = new ChassisSpeeds(
        MathUtil.clamp(x, -MAX_X_SPEED, MAX_X_SPEED), 
        MathUtil.clamp(y, -MAX_Y_SPEED, MAX_Y_SPEED), 
        MathUtil.clamp(theta, -MAX_ROT_SPEED, MAX_ROT_SPEED)
      );
    }

    // convert chassis speeds to module states
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 0); // TODO

    // drive modules
    frontLeft.set(states[0]);
    frontRight.set(states[1]);
    backLeft.set(states[2]);
    backRight.set(states[3]);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = new SwerveModuleState[4]; // TODO
    states[0] = frontLeft.state(); // front left
    states[1] = frontRight.state(); // front right
    states[2] = backLeft.state(); // back left
    states[3] = backRight.state(); // back right

    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      states
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Field oriented", () -> this.fieldOriented, null);
  }
}
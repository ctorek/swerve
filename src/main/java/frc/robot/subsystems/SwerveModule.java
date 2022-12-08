// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static frc.robot.Constants.*;

/** Add your docs here. */
public class SwerveModule implements Sendable {
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotateEncoder;

    private final SparkMaxPIDController drivePID;
    private final SparkMaxPIDController rotatePID;

    public SwerveModule(CANSparkMax driveMotor, CANSparkMax rotateMotor) {
        this.driveMotor = driveMotor;
        this.rotateMotor = rotateMotor;

        driveEncoder = driveMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();

        drivePID = driveMotor.getPIDController();
        rotatePID = rotateMotor.getPIDController();

        drivePID.setSmartMotionMaxVelocity(
            // m/s to rpm
            MAX_LINEAR_SPEED * (60 / WHEEL_CIRCUMFERENCE), 
            0
        ); 
        drivePID.setSmartMotionMaxAccel(
            // m/s to rpm/s
            MAX_LINEAR_ACCEL * (60 / WHEEL_CIRCUMFERENCE), 
            0
        );

        drivePID.setP(0.075);
        drivePID.setI(0);
        drivePID.setD(0.075);
        drivePID.setFF(0);

        rotatePID.setSmartMotionMaxVelocity(
            // rad/s to rpm
            MAX_ROT_SPEED * (60 / (2 * Math.PI)), 
            0
        );
        rotatePID.setSmartMotionMaxAccel(
            // rad/s^2 to rpm/s
            MAX_ROT_ACCEL * (60 / (2 * Math.PI)), 
            0
        );

        rotatePID.setP(0.0075);
        rotatePID.setI(0);
        rotatePID.setD(0.01);
        rotatePID.setFF(0);

        // gear ratio
        rotateEncoder.setPositionConversionFactor(1.0/12.8);
    }

    /**
     * module states should already be desaturated and optimized before calling this method
     * 
     * @param next the desired state
     */
    public void set(SwerveModuleState next) {
        // m/s to rpm
        double velocity = next.speedMetersPerSecond;

        // radians to rotations
        double angle = next.angle.getRadians() / (2 * Math.PI);

        drivePID.setReference(velocity, ControlType.kSmartVelocity);
        rotatePID.setReference(angle, ControlType.kPosition);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotateEncoder.setPosition(0);
    }

    /**
     * @return current state of swerve module
     */
    public SwerveModuleState state() {
        return new SwerveModuleState(
            velocity(), 
            new Rotation2d(angle())
        );
    }

    /**
     * @return current velocity in meters per second
     */
    private double velocity() {
        // rpm to m/s
        double velocity = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity());
        velocity *= (WHEEL_CIRCUMFERENCE / (2 * Math.PI));

        return velocity;
    }

    /**
     * @return current angle in radians
     */
    private double angle() {
        // rotations to radians
        double angle = rotateEncoder.getPosition() * (2 * Math.PI);
        return angle;
    }

    private boolean connected() {
        boolean drive = !driveMotor.getFirmwareString().equals("v0.0.0");
        boolean rotate = !rotateMotor.getFirmwareString().equals("v0.0.0");

        return drive && rotate;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve module");

        builder.addDoubleProperty("Current velocity (m per s)", this::velocity, null);
        builder.addDoubleProperty("Current angle (deg)", () -> this.angle() * (180 / Math.PI), null);
        builder.addBooleanProperty("Module motors connected", this::connected, null);
    }
}

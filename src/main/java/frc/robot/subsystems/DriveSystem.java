// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

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

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    // drive
    frontLeftDrive = new CANSparkMax(Constants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    frontRightDrive = new CANSparkMax(Constants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
    backLeftDrive = new CANSparkMax(Constants.BACK_LEFT_DRIVE, MotorType.kBrushless);
    backRightDrive = new CANSparkMax(Constants.BACK_RIGHT_DRIVE, MotorType.kBrushless);

    // rotate
    frontLeftRotate = new CANSparkMax(Constants.FRONT_LEFT_ROTATE, MotorType.kBrushless);
    frontRightRotate = new CANSparkMax(Constants.FRONT_RIGHT_ROTATE, MotorType.kBrushless);
    backLeftRotate = new CANSparkMax(Constants.BACK_LEFT_ROTATE, MotorType.kBrushless);
    backRightRotate = new CANSparkMax(Constants.BACK_RIGHT_ROTATE, MotorType.kBrushless);
  }

  /**
   * convert wheel speeds to vectors
   * 
   * @param x -1 to 1
   * @param y -1 to 1
   * @param theta -1 to 1
   */  
  public List<Matrix<N2, N1>> speedsToWheelVectors(double x, double y, double theta) {
    // input vectors
    Matrix<N2, N1> inputs = new Matrix<>(N2.instance, N1.instance);
    
    // rotation vectors
    Matrix<N2, N1> frontLeft = new Matrix<>(N2.instance, N1.instance);
    Matrix<N2, N1> frontRight = new Matrix<>(N2.instance, N1.instance);
    Matrix<N2, N1> backLeft = new Matrix<>(N2.instance, N1.instance);
    Matrix<N2, N1> backRight = new Matrix<>(N2.instance, N1.instance);

    frontLeft.set(1, 1, 0); // TODO
    frontLeft.set(2, 1, 0); // TODO

    frontRight.set(1, 1, 0); // TODO
    frontRight.set(2, 1, 0); // TODO

    backLeft.set(1, 1, 0); // TODO
    backLeft.set(2, 1, 0); // TODO

    backRight.set(1, 1, 0); // TODO
    backRight.set(2, 1, 0); // TODO

    // scale by rotation value
    frontLeft.times(theta);
    frontRight.times(theta);
    backLeft.times(theta);
    backRight.times(theta);

    frontLeft.plus(inputs);
    frontRight.plus(inputs);
    backLeft.plus(inputs);
    backRight.plus(inputs);

    List<Matrix<N2, N1>> vectors = new ArrayList<>(); // TODO
    vectors.add(frontLeft);
    vectors.add(frontRight);
    vectors.add(backLeft);
    vectors.add(backRight);

    return vectors;
  }

  public List<Matrix<N2, N1>> normalize(List<Matrix<N2, N1>> vectors) {
    List<Double> velocities = new ArrayList<>();
    for (Matrix<N2, N1> vec : vectors) {
      // extract velocities to separate list
      velocities.add(vec.get(0, 0));
    }

    // find the max absolute value of velocity for all the wheels
    double maxVel = Collections.max(
      velocities
      .stream()
      .map(Math::abs)
      .collect(Collectors.toList())
    );

    if (maxVel > 1) {
      // don't normalize if max value was within bounds already
      velocities = 
        velocities
        .stream()
        .map(v -> v / maxVel) // divide each by max
        .collect(Collectors.toList()); // put back into list
    }

    List<Matrix<N2, N1>> normalizedVelocities = new ArrayList<>();
    for (int i = 0; i < velocities.size(); i++) {
      // put velocities back into matrices
      var vector = vectors.get(i).copy();
      vector.set(0, 0, velocities.get(i));

      normalizedVelocities.add(vector);
    }

    return normalizedVelocities;
  }

  public List<Matrix<N2, N1>> optimize(List<Matrix<N2, N1>> prev, List<Matrix<N2, N1>> curr) {
    List<Double> deltas = new ArrayList<>();
    
    // find deltas for all angles
    for (int i = 0; i < prev.size(); i++) {
      double prevAngle = prev.get(i).get(2, 1);
      double currAngle = curr.get(i).get(2, 1);

      double delta = currAngle - prevAngle;
      deltas.add(delta);
    }

    

    return new ArrayList<>();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
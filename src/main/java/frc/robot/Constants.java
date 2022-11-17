// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drive motors
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_RIGHT_DRIVE = 4;

    // Rotate motors
    public static final int FRONT_LEFT_ROTATE = 5;
    public static final int FRONT_RIGHT_ROTATE = 6;
    public static final int BACK_LEFT_ROTATE = 7;
    public static final int BACK_RIGHT_ROTATE = 8;

    /** Module distance from center */
    public static final double MODULE_DIST = Units.inchesToMeters(11.25);

    /** Circumference of module wheels */
    public static final double WHEEL_CIRCUMFERENCE = 0;

    // Maximum speeds along each axis

    /** m/s */
    public static final double MAX_LINEAR_SPEED = 2.0;

    /** m/s^2 */
    public static final double MAX_LINEAR_ACCEL = 1.0;

    /** rad/s */
    public static final double MAX_ROT_SPEED = 1.0;

    /** rad/s^2 */
    public static final double MAX_ROT_ACCEL = 1.0;
}

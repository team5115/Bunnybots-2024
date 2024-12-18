package frc.team5115;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    private static final boolean isReplay = false;
    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final byte ARM_MOTOR_ID = 9;
    public static final byte DISPENSER_MOTOR_ID = 10;
    public static final byte SENSOR_ID = 9;

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static final double FIELD_WIDTH_METERS = 16.46;

    public static class SwerveConstants {
        public static final byte BACK_LEFT_DRIVE_ID = 2;
        public static final byte FRONT_LEFT_DRIVE_ID = 4;
        public static final byte BACK_RIGHT_DRIVE_ID = 6;
        public static final byte FRONT_RIGHT_DRIVE_ID = 8;

        public static final byte BACK_LEFT_TURN_ID = 1;
        public static final byte FRONT_LEFT_TURN_ID = 3;
        public static final byte BACK_RIGHT_TURN_ID = 5;
        public static final byte FRONT_RIGHT_TURN_ID = 7;

        public static final double MAX_LINEAR_SPEED = 4.2; // meters per second
        public static final double TRACK_WIDTH_X = .595; // meters
        public static final double TRACK_WIDTH_Y = .595; // meters
        public static final double DRIVE_BASE_RADIUS =
                Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        // Required for inverse kinematics. +x is forward, +y is left
        // The module order, as with everywhere else, is FL, FR, BL, BR
        public static final Translation2d[] MODULE_TRANSLATIONS =
                new Translation2d[] {
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / -2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / -2.0)
                };

        public static final Rotation2d FRONT_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(270);
        public static final Rotation2d FRONT_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BACK_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(180);
        public static final Rotation2d BACK_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(90);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear
        // 15 teeth on the bevel pinion, 13 teeth on the driving motor
        public static final double DrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);

        public static final int DrivingMotorCurrentLimit = 40; // amps
        public static final int TurningMotorCurrentLimit = 20; // amps
    }

    public static class VisionConstants {
        public static final String cameraName = "FOV_75_Degrees";
        public static final double camYaw = Math.toRadians(+180.0);
        public static final double camPitch = Math.toRadians(-40.0);
        public static final double camRoll = Math.toRadians(+180.0);
        public static final double camZ = +0.57;
        public static final double camX = +18.0;
        public static final double camY = -19.5;
        public static final Transform3d robotToCam =
                new Transform3d(camX, camY, camZ, new Rotation3d(camRoll, camPitch, camYaw));
    }
}

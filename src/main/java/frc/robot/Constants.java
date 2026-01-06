package frc.robot;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class InputConstants {
        public static final int K_XBOX_PORT = 0;
    }

    public static class DriveConstants {
        public static final double K_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double K_AUTO_SPEED = K_MAX_SPEED * 0.5d;
        public static final double K_MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        // TODO: Find reasonable tolerances with physical robot
        public static final double K_AUTO_TRANSLATION_TOLERANCE = 0.005; // in meters
        public static final double K_AUTO_ROTATION_TOLERANCE = 0.05; // in degrees
    }

    public static class FieldConstants {
        public static final double K_FIELD_WIDTH = 8.0518; // in meters, the shorter one, or the y direction
        public static final double K_FIELD_LENGTH = 17.548225; // in meters, the longer one, or the x direction
    }
}

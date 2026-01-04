package frc.robot;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class InputConstants {
        public static final int K_XBOX_PORT = 0;
    }

    public static class DriveConstants {
        public static final double K_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double K_MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }
}

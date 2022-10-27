package frc.robot;

public class Constants {
    public static final boolean debug = true;

    public static class DriveTrain {
        public static final double kP = 0.00075;
        public static final double kI = 0;
        public static final double kD = 0.0000;
        public static final double kIz = 0;
        public static final double kFF = 0.0000035;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final double maxRPM = 5200;
        public static final double setpointThreshold = 300;

        public static class TeleOP {
            public static final double leftMultiplication = 1;
            public static final double rightMultiplication = 1;
        }
    }

    public static class Turret {
        public static final double leftLimit = -90.0;
        public static final double rightLimit = 50.0;

        public static class Shooter {
            public static final double[] distances = { 47.5, 69.4, 77.0, 85.0, 97.0, 101.0, 178.0, 195.9, 224.0, 250.0,
                    12384 };
            public static final int[] wheelValues = { 6650, 6800, 7000, 7200, 7300, 7650, 8200, 9100, 9800, 10500,
                    10000 };
            public static final double power = 0.6; // step 1: 0.4
            public static final double tolerance = 100;
        }

        public static class HoodPoint {
            public static final double[] distances = Shooter.distances;
            public static final int[] hoodValues = { 30, 42, 53, 55, 60, 62, 90, 95, 110, 102, 100 };
        }

        public static class HoodShooter {
            public static final double[] distances = {};
            public static final int[] wheelValues = {};
        }

        public static class Default {
            public static final boolean aim = false;
        }

        public static class Fender {
            public static final double hoodShooterMotorSpeed = 9000;
            public static final int setPointHood = 11;
            public static final int setPointRotation = 0;
            public static final int setPointShooterPID = 6200;
        }
    }

    public static class Climber {
        public static final double shoulderUnlatchSpot = -4.5;
        public static final double primaryUnlatchSpot = -1000.0;
        public static final int unlatchSpeedPrimary = 0;
        public static final double unlatchSpeedShoulder = -0.3;
    }
}

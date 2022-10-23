package frc.robot;

public class Constans {
    public static final boolean debug = false;

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
        public static class HoodShooter {
            public static final double[] distances = { 44.0, 77, 113.4, 145.5, 170.8, 220.5 };
            public static final double[] wheelValues = { 4000, 4000, 4000, 4000, 4000, 4000 };
        }

        public static class HoodPoint {
            public static final double[] distances = { 55.0, 60, 77, 100, 120, 145, 167, 202.95, 244.77, 305.66 };
            public static final int[] hoodValues = { 5, 40, 60, 80, 90, 95, 102, 115, 135, 140 };
        }

        public static class Default {
            public static final boolean aim = false;
        }

        public static class Fender {
            public static final double hoodShooterMotorSpeed = 9000;
            public static final int setPointHood = 0;
            public static final int setPointRotation = 0;
            public static final int setPointShooterPID = 9000;
        }
    }

    public static class Climber {
        public static final double shoulderUnlatchSpot = -3.5;
        public static final double primaryUnlatchSpot = -3.5;
        public static final int unlatchSpeedPrimary = -1;
        public static final double unlatchSpeedShoulder = -0.5;
    }
}

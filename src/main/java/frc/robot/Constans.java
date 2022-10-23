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

    public static class Shooter {
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

package frc.robot.subsystems.BallPath.Shooter;

import java.util.concurrent.TimeUnit;

// import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterImpl extends RepeatingPooledSubsystem implements Shooter {
    // Define the shooter motor.
    public final WPI_TalonFX shooterMotor;
    public final VictorSPX hoodMotor;
    public final PIDController shooterPIDController;

    // PID Constants (as of February 17, 2022).
    double Kp = 0.000550;
    double Ki = 0.000055;
    double Kd = 0.000020;

    public ShooterImpl(WPI_TalonFX shooterMotor, VictorSPX hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
        this.shooterPIDController = new PIDController(Kp, Ki, Kd);
    }

    @Override
    public void defineResources() {
        require(shooterMotor);
        require(hoodMotor);
    }

    @Override
    public void task() {}

    @Override
    public void findAndCenterTarget() {}  // Perhaps this method is not required (given that one could simply utilize centerTarget() instead).

    @Override
    public void centerTarget(double tx) {
        // TODO: Move the shooter given a measurement.
    }

    @Override
    public double getDistance(double angle2) {
        // Calculate the height difference between the robot and the top of the target.
        double limelightHeight = 0;
        double targetHeight = 0;
        double heightDifference = targetHeight - limelightHeight;

        // Calculate the total angle (mounting angle + ty).
        double mountingAngle = 0;
        double totalAngle = Math.toRadians(mountingAngle + angle2);

        // Calculate the horizontal distance between the limelight and the target.
        double distance = Math.tan(heightDifference / totalAngle);
        return distance;
    }

    // Runs flywheel.
    @Override
    public boolean readyToShoot() {
        return true;
    }

    @Override
    public int checkBalls(){
        return 0;
    }

    @Override
    public void setHoodAngle(double distance) {
        double encoderReadingPosition = this.hoodMotor.getSelectedSensorPosition();

        while (encoderReadingPosition > -600000) {
            hoodMotor.set(ControlMode.PercentOutput, -.5);
            encoderReadingPosition = this.hoodMotor.getSelectedSensorPosition();
          }

        hoodMotor.set(ControlMode.PercentOutput, 0); 
    }

    @Override
    public void start(double distance) {
        // TODO: Given the horizontal distance to the target, utilize a certain set point value.
        double currentOutput = this.shooterPIDController.calculate(this.shooterMotor.getSelectedSensorVelocity(), 0);  // The second argument is a set point--to be set later.
        this.shooterMotor.set(currentOutput);
    }

    @Override
    public void stop() {
        this.shooterMotor.set(0);
    }
}

package frc.robot.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

public class RawDriveImpl extends RepeatingPooledSubsystem implements Drive {

    // motor controller groups
    private CANSparkMax leftSide;
    private CANSparkMax rightSide;
    
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    Pose2d pose;

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    public RawDriveImpl(CANSparkMax leftSide, CANSparkMax rightSide, RelativeEncoder leftEncoder, RelativeEncoder rightEncoder) {
        super(20, TimeUnit.MILLISECONDS);
        // basic drivetrain stuff
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder; 
    }

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.66));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());


    @Override
    public void defineResources() {
        require(leftSide);
        require(rightSide);
        
        require(leftEncoder);
        require(rightEncoder);
    }

    public double lSpeed(){
      return leftSide.getEncoder().getVelocity()/5*2*Math.PI*Units.inchesToMeters(2)/60;
    }

    public double rSpeed(){
      return rightSide.getEncoder().getVelocity()/5*2*Math.PI*Units.inchesToMeters(2)/60;
    }

    @Override
    public void task() throws Exception {
        pose = odometry.update(getHeading(), lSpeed(), rSpeed());
    }

    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(-this.gyro.getAngle());
    }

    public static WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation, boolean squareInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    
        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        // if (squareInputs) {
        //   xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        //   zRotation = Math.copySign(zRotation * zRotation, zRotation);
        // }
    
        double leftSpeed;
        double rightSpeed;
    
        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    
        if (xSpeed >= 0.0) {
          // First quadrant, else second quadrant
          if (zRotation >= 0.0) {
            leftSpeed = maxInput;
            rightSpeed = xSpeed - zRotation;
          } else {
            leftSpeed = xSpeed + zRotation;
            rightSpeed = maxInput;
          }
        } else {
          // Third quadrant, else fourth quadrant
          if (zRotation >= 0.0) {
            leftSpeed = xSpeed + zRotation;
            rightSpeed = maxInput;
          } else {
            leftSpeed = maxInput;
            rightSpeed = xSpeed - zRotation;
          }
        }
    
        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }
    
        return new WheelSpeeds(leftSpeed, rightSpeed);
      }

    public void drive(double leftSpeed, double rotation){
        var speeds = arcadeDriveIK(leftSpeed, rotation, true);

        leftSide.set(speeds.left);
        rightSide.set(speeds.right);
    }

    @Override
    public Pair<Double, Double> getEncoderTicks(){
      return Pair.of(this.leftEncoder.getPosition(), this.rightEncoder.getPosition());
    }


    @Override
    public void resetEncoderTicks() {
        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);
    }
    
    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
      switch (current) {
        case ON_INIT:
        case ON_AUTO:
        case ON_TELEOP:
        case ON_TEST:
          this.start();
          break;
        case ON_DISABLED:
        case NONE:
        default:
          this.leftEncoder.setPosition(0);
          this.rightEncoder.setPosition(0);
          this.cancel();
          break;
    }
  }

}
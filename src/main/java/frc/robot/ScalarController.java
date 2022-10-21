package frc.robot;

public class ScalarController {
    private double scale;

    public ScalarController(double scale) {
        this.scale = scale;
    }

    public boolean atSetpoint() {
        return true;
    }

    public double calculate(double measurement, double setpoint) {
      return setpoint / scale;
    }
}

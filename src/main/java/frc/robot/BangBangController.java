package frc.robot;

public class BangBangController extends WPIBangBangController {

    protected final double power;

    public BangBangController(double power, double tolerance) {
        super(tolerance);
        this.power = power;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
      double original = super.calculate(measurement, setpoint);
      return original == 1 ? power : 0;
    }
}

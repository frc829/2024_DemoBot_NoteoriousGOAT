package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Spinner;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.utility.GoatMath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransportSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 24;
    private static final Measure<Voltage> kS = Volts.of(0.14877);
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 0.0010622 / 12.0;
  }

  public final Measure<Voltage> voltage;
  public final Measure<Dimensionless> velocity;
  public final Consumer<Double> spin;
  public final Runnable stop;
  public final Runnable update;

  private TransportSubsystem(
      Measure<Voltage> voltage,
      Measure<Dimensionless> velocity,
      Consumer<Double> spin,
      Runnable stop,
      Runnable update) {
    this.voltage = voltage;
    this.velocity = velocity;
    this.spin = spin;
    this.stop = stop;
    this.update = update;

    Command defaultCommand = run(stop);
    defaultCommand.setName("STOP");
    this.setDefaultCommand(defaultCommand);

  }

  @Override
  public void periodic() {
    update.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Velocity",
        () -> GoatMath.round(velocity.in(Percent), 2),
        null);

    builder.addDoubleProperty(
        "Voltage",
        () -> GoatMath.round(voltage.in(Volts), 2),
        null);
  }

  public static final Supplier<TransportSubsystem> create = () -> {
    CANSparkMax canSparkMax = new CANSparkMax(Constants.deviceId, MotorType.kBrushless);
    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(canSparkMax, DCMotor.getNeo550(1));
    } 
    canSparkMax.setIdleMode(IdleMode.kBrake);
    canSparkMax.getPIDController().setP(Constants.slot0kP, 0);
    canSparkMax.getPIDController().setI(Constants.slot0kI, 0);
    canSparkMax.getPIDController().setD(Constants.slot0kD, 0);
    canSparkMax.getPIDController().setFF(Constants.slot0kF, 0);
    canSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

    Spinner spinner = Motor.REV.createNEO550Motor
        .andThen(Spinner.create)
        .apply(canSparkMax, Constants.kS);

    return new TransportSubsystem(
        spinner.voltage,
        spinner.velocity,
        spinner.spin,
        spinner.stop,
        spinner.update);
  };
}

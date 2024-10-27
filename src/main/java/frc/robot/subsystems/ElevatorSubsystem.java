package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Elevator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.utility.GoatMath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 29;
    private static final double gearing = 4.0 * 4.0 * 3.0 * 40.0 / 30.0;
    private static final Measure<Distance> drumRadius = Inches.of(1.0);
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
    private static final double slot1kP = 2;
    private static final double slot1kI = 0.0;
    private static final double slot1kD = 0.0;
    private static final double slot1kF = 0.0;
  }

  public final Measure<Voltage> voltage;
  public final Measure<Distance> position;
  public final Measure<Dimensionless> velocity;
  public final Consumer<Measure<Distance>> move;
  public final Consumer<Double> drive;
  public final Runnable hold;
  public final Runnable update;

  private ElevatorSubsystem(
      Measure<Voltage> voltage,
      Measure<Distance> position,
      Measure<Dimensionless> velocity,
      Consumer<Measure<Distance>> move,
      Consumer<Double> drive,
      Runnable hold,
      Runnable update) {
    this.voltage = voltage;
    this.position = position;
    this.velocity = velocity;
    this.move = move;
    this.drive = drive;
    this.hold = hold;
    this.update = update;

    Command defaultCommand = run(hold);
    defaultCommand.setName("HOLD");
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
        "Voltage",
        () -> GoatMath.round(voltage.in(Volts), 2),
        null);

    builder.addDoubleProperty(
        "Position",
        () -> GoatMath.round(position.in(Meters), 2),
        null);

    builder.addDoubleProperty(
        "Velocity",
        () -> GoatMath.round(velocity.in(Percent), 2),
        null);
  }

  public static final Supplier<ElevatorSubsystem> create = () -> {
    CANSparkMax neo = new CANSparkMax(Constants.deviceId, MotorType.kBrushless);
    neo.getPIDController().setP(Constants.slot0kP, 0);
    neo.getPIDController().setI(Constants.slot0kI, 0);
    neo.getPIDController().setD(Constants.slot0kD, 0);
    neo.getPIDController().setFF(Constants.slot0kF, 0);
    neo.getPIDController().setP(Constants.slot1kP, 1);
    neo.getPIDController().setI(Constants.slot1kI, 1);
    neo.getPIDController().setD(Constants.slot1kD, 1);
    neo.getPIDController().setFF(Constants.slot1kF, 1);
    neo.setIdleMode(IdleMode.kBrake);
    neo.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

    if(RobotBase.isSimulation()){
      REVPhysicsSim.getInstance().addSparkMax(neo, DCMotor.getNEO(1));
    }

    Elevator elevator = Motor.REV.createNEOMotor
        .andThen(Elevator.create
            .apply(Constants.gearing)
            .apply(Constants.drumRadius))
        .apply(neo, Volts.of(0.0));

    return new ElevatorSubsystem(
        elevator.voltage,
        elevator.position,
        elevator.velocity,
        elevator.move,
        elevator.drive,
        elevator.hold,
        elevator.update);

  };
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Turner;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.utility.GoatMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTiltSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 18;
    private static final double gearing = 5.0 * 4.0 * 3.0 * 56.0 / 18.0;
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
    private static final double slot1kP = 1.0;
    private static final double slot1kI = 0.0;
    private static final double slot1kD = 0.0;
    private static final double slot1kF = 0.0;
  }

  public final Measure<Voltage> voltage;
  public final Measure<Angle> angle;
  public final Measure<Angle> absoluteAngle;
  public final Measure<Dimensionless> velocity;
  public final Consumer<Measure<Angle>> setRelativeEncoder;
  public final Consumer<Measure<Angle>> turn;
  public final Consumer<Double> drive;
  public final Runnable resetRelEncoderFromAbsolute;
  public final Runnable hold;
  public final Runnable stop;
  public final Runnable update;

  private ShooterTiltSubsystem(
      Measure<Voltage> voltage,
      Measure<Angle> angle,
      Measure<Angle> absoluteAngle,
      Measure<Dimensionless> velocity,
      Consumer<Measure<Angle>> setRelativeEncoder,
      Consumer<Measure<Angle>> turn,
      Consumer<Double> drive,
      Runnable resetRelEncoderFromAbsolute,
      Runnable hold,
      Runnable stop,
      Runnable update) {
    this.voltage = voltage;
    this.angle = angle;
    this.absoluteAngle = absoluteAngle;
    this.velocity = velocity;
    this.setRelativeEncoder = setRelativeEncoder;
    this.turn = turn;
    this.resetRelEncoderFromAbsolute = resetRelEncoderFromAbsolute;
    this.drive = drive;
    this.hold = hold;
    this.update = update;
    this.stop = stop;

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
        "Angle",
        () -> GoatMath.round(MathUtil.inputModulus(angle.in(Degrees), -180, 180), 2),
        null);

    builder.addDoubleProperty(
        "Absolute Angle",
        () -> GoatMath.round(absoluteAngle.in(Degrees), 2),
        null);

    builder.addDoubleProperty(
        "Velocity",
        () -> GoatMath.round(velocity.in(Percent), 2),
        null);

    builder.addDoubleProperty("Angle Diff",
        () -> {
          double angleNum = GoatMath.round(MathUtil.inputModulus(angle.in(Degrees), -180, 180), 2);
          double absoluteAngleNum = GoatMath.round(absoluteAngle.in(Degrees), 2);
          return Math.abs(angleNum - absoluteAngleNum);
        }, null);
  }

  public static final Supplier<ShooterTiltSubsystem> create = () -> {

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
    neo.setInverted(true);
    neo.getPIDController().setPositionPIDWrappingEnabled(true);
    neo.getPIDController().setPositionPIDWrappingMinInput(-0.5 * Constants.gearing);
    neo.getPIDController().setPositionPIDWrappingMaxInput(0.5 * Constants.gearing);
    neo.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(neo, DCMotor.getNEO(1));
    }

    Turner turner = Motor.REV.createNEOMotor
        .andThen(Turner.create.apply(Constants.gearing)).apply(neo, Volts.of(0.0));

    return new ShooterTiltSubsystem(
        turner.voltage,
        turner.angle,
        turner.absoluteAngle,
        turner.velocity,
        turner.setRelativeEncoder,
        turner.turn,
        turner.drive,
        turner.resetRelEncoderFromAbsolute,
        turner.hold,
        turner.stop,
        turner.update);
  };
}

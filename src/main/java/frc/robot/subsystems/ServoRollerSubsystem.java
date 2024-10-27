package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoRollerSubsystem extends SubsystemBase {

  public static final class Constants {
    private static final int channel = 0;
    private static final int max = 2500;
    private static final int deadbandMax = 2497;
    private static final int center = 1500;
    private static final int deadbandMin = 503;
    private static final int min = 500;
    private static final double runningDegree = 270.0;
  }

  private final Servo servo;
  private final PWMSim servoSim;

  public ServoRollerSubsystem() {
    servo = new Servo(Constants.channel);
    servo.setBoundsMicroseconds(
        Constants.max,
        Constants.deadbandMax,
        Constants.center,
        Constants.deadbandMin,
        Constants.min);

    servoSim = new PWMSim(servo);
  }

  private double getPosition() {
    if (RobotBase.isSimulation()) {
      return servoSim.getPosition();
    } else {
      return servo.getPosition();
    }
  }

  private double getSpeed() {
    if (RobotBase.isSimulation()) {
      return servoSim.getSpeed();
    } else {
      return servo.getSpeed();
    }
  }

  private void setPosition(double position) {
    if (RobotBase.isSimulation()) {
      servoSim.setPosition(position);
    } else {
      servo.setPosition(position);
    }
  }

  private void setAngle(double angleDegrees) {
    if (angleDegrees >= 0 && angleDegrees <= Constants.runningDegree) {
      double position = angleDegrees / Constants.runningDegree;
      setPosition(position);
    }
  }

  public void setSpeed(double speed) {
    if (RobotBase.isSimulation()) {
      servoSim.setSpeed(speed);
    } else {
      servo.setSpeed(speed);
    }
  }

  public void stop() {
    if (RobotBase.isSimulation()) {
      servoSim.setSpeed(0.0);
    } else {
      servo.setDisabled();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Servo Position (deg)",
        () -> getPosition() * Constants.runningDegree,
        null);
    builder.addDoubleProperty(
        "Servo Speed",
        () -> getSpeed(),
        null);
  }

  public Command createStopCommand() {
    return run(this::stop);
  }

  public Command createSetAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class MechanismSubsystem extends SubsystemBase {

  private MechanismLigament2d elevatorShaft;
  private MechanismLigament2d shooter;

  public MechanismSubsystem() {
    Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("bottom", 1.5, 0.5);
    elevatorShaft = root.append(new MechanismLigament2d("elevator", 0.0, 75));
    elevatorShaft.setLineWeight(1.0);
    elevatorShaft.setColor(new Color8Bit("#FFFFFF"));

    var elevatorBar = root.append(new MechanismLigament2d("elevatorBar", 1.0, 75));
    elevatorBar.setLineWeight(1.0);
    elevatorBar.setColor(new Color8Bit("#FFFFFF"));
    shooter = elevatorShaft.append(new MechanismLigament2d("shooter", 0.5, -75));
    shooter.setLineWeight(1.0);
    shooter.setColor(new Color8Bit("#FFFF00"));
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Mech", mech);

    }

  }

  @Override
  public void periodic() {
    elevatorShaft.setLength(RobotContainer.elevatorSubsystem.position.in(Meters));
    shooter.setAngle(RobotContainer.shooterTiltSubsystem.angle.in(Degrees) - 75);
  }
}

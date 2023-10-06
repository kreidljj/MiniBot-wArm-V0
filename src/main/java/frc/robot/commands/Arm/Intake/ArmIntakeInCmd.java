package frc.robot.commands.Arm.Intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Secondary.ArmSubsystem;

public class ArmIntakeInCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmIntakeInCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.secondaryJoystick.setRumble(RumbleType.kRightRumble, 0.5);
    }

    @Override
    public void execute() {
        armSubsystem.intakeMotor.set(Constants.ArmConstants.gIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeMotor.set(0.07);
        RobotContainer.secondaryJoystick.setRumble(RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
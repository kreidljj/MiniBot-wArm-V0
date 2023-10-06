package frc.robot.subsystems.Secondary;

// import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    public CANSparkMax leftArmSlider;
    public CANSparkMax rightArmSlider;

    public CANSparkMax suck;

    public RelativeEncoder sliderEncoder;
    public RelativeEncoder grabberEncoder;

    public CANSparkMax grabberMotor;

    // public CANSparkMax armRotateMotor;
    public CANSparkMax wristRotateMotor;
    public CANSparkMax intakeMotor;

    // public static SparkMaxAbsoluteEncoder armRotateEncoder;
    // public SparkMaxAbsoluteEncoder armRotateEncoder;
    public SparkMaxAbsoluteEncoder wristRotateEncoder;


    public ArmSubsystem() {
        //leftArmSlider = new CANSparkMax(Constants.ArmConstants.kLeftSliderMotor, MotorType.kBrushless);
        //rightArmSlider = new CANSparkMax(Constants.ArmConstants.kRightSliderMotor, MotorType.kBrushless);

        //wristRotateMotor =  new CANSparkMax(Constants.ArmConstants.kManipulatorWristMotor, MotorType.kBrushless);
        intakeMotor =  new CANSparkMax(Constants.ArmConstants.kManipulatorIntakeMotor, MotorType.kBrushless);
        // armRotateMotor = new CANSparkMax(Constants.ArmConstants.kArmRotateMotor,MotorType.kBrushless);

        sliderEncoder = leftArmSlider.getEncoder();

        // armRotateEncoder = armRotateMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        // armRotateEncoder = armRotateMotor.	getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        // armRotateEncoder.setPositionConversionFactor(360);
        // armRotateEncoder.setZeroOffset(190);

        // grabberEncoder = grabberMotor.getEncoder();

        wristRotateEncoder = wristRotateMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        wristRotateEncoder.setPositionConversionFactor(360);
        wristRotateEncoder.setZeroOffset(259);

        // armRotateEncoder.setPositionConversionFactor(0);
        sliderEncoder.setPositionConversionFactor(0.5855165417); // 1.31741221882
        // grabberEncoder.setPositionConversionFactor(2.666);
    }
}
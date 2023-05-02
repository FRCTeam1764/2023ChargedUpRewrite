package frc.robot.SwerveConstants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.SwerveConstants.driveConstants;
public final class CTREConfigs {
    public static TalonFXConfiguration swerveAngleFXConfig;
    public static TalonFXConfiguration swerveDriveFXConfig;
    public static CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            driveConstants.Swerve.angleEnableCurrentLimit, 
            driveConstants.Swerve.angleContinuousCurrentLimit, 
            driveConstants.Swerve.anglePeakCurrentLimit, 
            driveConstants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = driveConstants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = driveConstants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = driveConstants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = driveConstants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            driveConstants.Swerve.driveEnableCurrentLimit, 
            driveConstants.Swerve.driveContinuousCurrentLimit, 
            driveConstants.Swerve.drivePeakCurrentLimit, 
            driveConstants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = driveConstants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = driveConstants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = driveConstants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = driveConstants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = driveConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = driveConstants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = driveConstants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
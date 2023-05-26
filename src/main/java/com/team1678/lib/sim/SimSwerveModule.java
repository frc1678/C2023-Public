package com.team1678.lib.sim;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1678.frc2023.Constants;
import com.team1678.lib.Conversions;
import com.team1678.lib.Util;
import com.team1678.lib.swerve.ISwerveModule;
import com.team1678.lib.swerve.ModuleState;
import com.team1678.lib.swerve.SwerveModule.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class SimSwerveModule implements ISwerveModule {
    public int moduleNumber;
    public double angleOffset;
    private WPI_TalonFX mAngleMotor;
    private WPI_TalonFX mDriveMotor;
    // private WPI_CANCoder angleEncoder;
    private double lastAngle;

    private double anglekP;
    private double anglekI;
    private double anglekD;

    public SimSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        // angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID);
        // configAngleEncoder();
        // angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        // angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

        /* Angle Motor Config */
        mAngleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        TalonFXConfiguration angleConfiguration = Constants.SwerveConstants.swerveAngleFXConfig();
        anglekP = angleConfiguration.slot0.kP;
        anglekI = angleConfiguration.slot0.kI;
        anglekD = angleConfiguration.slot0.kD;

        /* Drive Motor Config */
        mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();

        // Add to physics sim
        PhysicsSim.getInstance().addTalonFX(mAngleMotor, 0.0, 6800);
        PhysicsSim.getInstance().addTalonFX(mDriveMotor, 0.0, 10000);
    }

    @Override
    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop) {
        // desiredState = desiredState.optimize(Rotation2d.fromDegrees(lastAngle));
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity);
        }

        double angle =  desiredState.angle.getDegrees();
        angle = Util.placeInAppropriate0To360Scope(getState().angle.getDegrees(), angle);
        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Constants.SwerveConstants.swerveAngleFXConfig());
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Constants.SwerveConstants.swerveDriveFXConfig());
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public void updateAnglePID(double kP, double kI, double kD) {
        if (anglekP != kP) {
            anglekP = kP;
            mAngleMotor.config_kP(0, anglekP, Constants.kLongCANTimeoutMs);
        }
        if (anglekI != kI) {
            anglekI = kI;
            mAngleMotor.config_kI(0, anglekI, Constants.kLongCANTimeoutMs);
        }
        if (anglekD != kP) {
            anglekD = kD;
            mAngleMotor.config_kD(0, anglekD, Constants.kLongCANTimeoutMs);
        }
    }

    public double[] getAnglePIDValues() {
        double[] values = { anglekP, anglekI, anglekD };
        return values;
    }

    @Override
    public Rotation2d getCanCoder() {
        return new Rotation2d();
        // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getTargetAngle() {
        return lastAngle;
    }

    @Override
    public ModuleState getState() {
        double distance = Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.angleGearRatio));
        return new ModuleState(distance, angle, velocity);
    }

    public SwerveModuleState getSpeed() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    @Override
    public int moduleNumber() {
        return moduleNumber;
    }

    @Override
    public double moduleOffset() {
        return angleOffset;
    }

    @Override
    public double angleOffset() {
        return 0;
    }
}

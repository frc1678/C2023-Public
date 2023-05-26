package com.team1678.lib.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.subsystems.Subsystem;
import com.team1678.lib.Conversions;
import com.team1678.lib.Util;
import com.team1678.lib.logger.Log;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    private final int kModuleNumber;
    private final double kAngleOffset;

    private TalonFX mAngleMotor;
    private com.ctre.phoenixpro.hardware.TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private ModuleState targetModuleState;

    private static SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConstants.angleEnableCurrentLimit,
            Constants.SwerveConstants.angleContinuousCurrentLimit,
            Constants.SwerveConstants.anglePeakCurrentLimit,
            Constants.SwerveConstants.anglePeakCurrentDuration);

    private static TalonFXConfiguration neutralCoastConfigs = Constants.SwerveConstants.swerveDriveProFXConfig();
    private static TalonFXConfiguration neutralBrakeConfigs = Constants.SwerveConstants.swerveDriveProFXConfig();

    static {
        neutralCoastConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        neutralBrakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.kModuleNumber = moduleNumber;
        kAngleOffset = moduleConstants.angleOffset;

        // Absolute encoder config
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "canivore1");
        angleEncoder.configFactoryDefault(Constants.kLongCANTimeoutMs);
        angleEncoder.configAllSettings(Constants.SwerveConstants.swerveCancoderConfig(), Constants.kLongCANTimeoutMs);
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, Constants.kLongCANTimeoutMs);
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100, Constants.kLongCANTimeoutMs);

        // Angle motor config
        mAngleMotor = TalonFXFactory.createDefaultTalon(moduleConstants.angleMotorID, "canivore1");
        mAngleMotor.configSupplyCurrentLimit(angleSupplyLimit, Constants.kLongCANTimeoutMs);
        mAngleMotor.config_kP(0, Constants.SwerveConstants.angleKP, Constants.kLongCANTimeoutMs);
        mAngleMotor.config_kI(0, Constants.SwerveConstants.angleKI, Constants.kLongCANTimeoutMs);
        mAngleMotor.config_kD(0, Constants.SwerveConstants.angleKD, Constants.kLongCANTimeoutMs);
        mAngleMotor.config_kF(0, Constants.SwerveConstants.angleKF, Constants.kLongCANTimeoutMs);
        mAngleMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.kLongCANTimeoutMs);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        mAngleMotor.setSelectedSensorPosition(0);

        // Drive motor config
        mDriveMotor = new com.ctre.phoenixpro.hardware.TalonFX(moduleConstants.driveMotorID, "canivore1");
        mDriveMotor.getConfigurator().apply(neutralBrakeConfigs,
                Constants.kLongCANTimeoutMs);
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setRotorPosition(0.0);

        resetToAbsolute();
    }

    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop) {
        targetModuleState = desiredState;

        double targetAngle = targetModuleState.angle.getDegrees();
        Rotation2d currentAngle = Rotation2d.fromDegrees(mPeriodicIO.rotationPosition);


        if(isOpenLoop) {
            mPeriodicIO.targetVelocity = targetModuleState.speedMetersPerSecond;
        } else {
            mPeriodicIO.targetVelocity = Util.limit(targetModuleState.speedMetersPerSecond, Constants.SwerveConstants.maxAttainableSpeed);
        }

        if (Util.shouldReverse(Rotation2d.fromDegrees(targetAngle), currentAngle)) {
            mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
            targetAngle += 180.0;
        }

        targetAngle = Util.placeInAppropriate0To360Scope(getCurrentUnboundedDegrees(), targetAngle);

        mPeriodicIO.rotationDemand = Conversions.degreesToFalcon(targetAngle,
                Constants.SwerveConstants.angleGearRatio);

        if (isOpenLoop) {
            mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
            mPeriodicIO.driveDemand = mPeriodicIO.targetVelocity / Constants.SwerveConstants.maxSpeed;
        } else {
            mPeriodicIO.driveControlMode = ControlMode.Velocity;
            mPeriodicIO.driveDemand = Conversions.MPSToRPS(mPeriodicIO.targetVelocity,
                    Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        }
    }

    public void resetToAbsolute() {
        double angle = Util.placeInAppropriate0To360Scope(mPeriodicIO.rotationPosition, getCanCoder().getDegrees() - kAngleOffset);
        double absolutePosition = Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public ModuleState getState() {
        return new ModuleState(mPeriodicIO.drivePosition, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mPeriodicIO.rotationPosition), mPeriodicIO.velocity);
    }

    @Override
    public synchronized void readPeriodicInputs() {

        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.velocity = Conversions.RPSToMPS(mDriveMotor.getRotorVelocity().getValue(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);

        mPeriodicIO.rotationPosition = Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.angleGearRatio);

        mPeriodicIO.drivePosition = Conversions.rotationsToMeters(mDriveMotor.getRotorPosition().getValue(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);;
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        double targetAngle = targetModuleState.angle.getDegrees();
        Rotation2d currentAngle = Rotation2d.fromDegrees(mPeriodicIO.rotationPosition);
        if (Util.shouldReverse(Rotation2d.fromDegrees(targetAngle), currentAngle)) {
            mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
            targetAngle += 180.0;
        }
        targetAngle = Util.placeInAppropriate0To360Scope(getCurrentUnboundedDegrees(), targetAngle);

        mPeriodicIO.rotationDemand = Conversions.degreesToFalcon(targetAngle,
                Constants.SwerveConstants.angleGearRatio);

        mAngleMotor.set(ControlMode.Position, mPeriodicIO.rotationDemand);
        if (mPeriodicIO.driveControlMode == ControlMode.Velocity) {
            mDriveMotor.setControl(new VelocityTorqueCurrentFOC(mPeriodicIO.driveDemand));
        } else {
            mDriveMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false));
        }
    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final double angleOffset;

        public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
        }
    }

    public static class mPeriodicIO {
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double rotationPosition = 0.0;
        public double drivePosition = 0.0;
        public double velocity = 0.0;

        // Outputs
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double rotationDemand;
        public double driveDemand;
    }

    public int moduleNumber() {
        return kModuleNumber;
    }

    public double angleOffset() {
        return kAngleOffset;
    }

    public double getDriveMotorCurrent() {
        return mDriveMotor.getStatorCurrent().getValue();
    }

    public void setDriveNeutralBrake(boolean wantBrake) {
        if (wantBrake) {
            mDriveMotor.getConfigurator().apply(neutralBrakeConfigs, Constants.kCANTimeoutMs);
            mAngleMotor.setNeutralMode(NeutralMode.Coast);
        } else {
            mDriveMotor.getConfigurator().apply(neutralCoastConfigs, Constants.kCANTimeoutMs);
            // Brake angle motors when coasting drive
            mAngleMotor.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Module" + kModuleNumber + " Angle Position", mPeriodicIO.rotationPosition);
        SmartDashboard.putNumber("Module" + kModuleNumber + " Drive Position", mPeriodicIO.drivePosition);
        SmartDashboard.putNumber("Module" + kModuleNumber + " Velocity", mPeriodicIO.velocity);
    }

    @Log
    public double getTargetAngle() {
        return Conversions.falconToDegrees(mPeriodicIO.rotationDemand,
                Constants.SwerveConstants.angleGearRatio);
    }

    @Log
    public double getTargetVelocity() {
        return mPeriodicIO.targetVelocity;
    }

    @Log
    public double getCurrentSpeed() {
        return getState().speedMetersPerSecond;
    }

    @Log
    public double getCurrentUnboundedDegrees() {
        return mPeriodicIO.rotationPosition;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}

package com.team1678.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Constants.WristConstants;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.lib.logger.Log;
import com.team1678.lib.requests.Request;
import com.team254.lib.util.Util;
import com.team1678.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends ServoMotorSubsystem {

    private static Wrist mInstance;

    public static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist(WristConstants.kWristConstants);
        }
        return mInstance;
    }

    private Wrist(final ServoMotorSubsystemConstants constants) {
        super(constants);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.overrideSoftLimitsEnable(false);
        mMaster.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, Constants.kLongCANTimeoutMs);
    }

    // Homing refers to moving the wrist into it's "zero" position. 
    private boolean mHoming = true;
    private final StatorCurrentLimitConfiguration kHomingConfig = new StatorCurrentLimitConfiguration(true, 40, 40, 0.02); 
    private final DelayedBoolean mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.2); 

    public void setWantHome(boolean home) {
        if (home) {
            mMaster.configStatorCurrentLimit(kHomingConfig, Constants.kCANTimeoutMs); 
        } else {
            mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                        mConstants.kEnableStatorCurrentLimit,
                        mConstants.kStatorContinuousCurrentLimit,
                        mConstants.kStatorPeakCurrentLimit,
                        mConstants.kStatorPeakCurrentDuration), Constants.kCANTimeoutMs); 
        }
        mHoming = home;
    }

    public synchronized void setWantJog(double delta) {
        setSetpointMotionMagic(getSetpointHomed() + delta);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                mMaster.setNeutralMode(NeutralMode.Brake);
                setSetpointMotionMagic(mConstants.kHomePosition);
            }

            @Override
            public void onLoop(double timestamp) {
                // constantly re-homing unless in open loop
                if (getSetpointHomed() >= mConstants.kHomePosition && (atHomingLocation()) && !mHoming) {
                    setWantHome(true);
                } else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
                    setWantHome(false);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mMaster.setNeutralMode(NeutralMode.Brake);
            }

        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.overrideSoftLimitsEnable(false);
        } else {
            mMaster.overrideSoftLimitsEnable(true);
        }

        if (mHoming) {
            if (mHomingDelay.update(Timer.getFPGATimestamp(),
                    Util.epsilonEquals(getVelocity(), 0.0, 5.0))) {
                zeroSensors();
                setOpenLoop(0.0);
                setWantHome(false);
            } else {
                setOpenLoop(1.0 / mConstants.kMaxVoltage);
            }
            super.writePeriodicOutputs();
        } else {
            super.writePeriodicOutputs();
        }
    }

    public Request wristRequest(double angle, boolean waitForPosition) {
        return new Request() {

            @Override
            public void act() {
                setSetpointMotionMagic(angle);
            }

            @Override
            public boolean isFinished() {
                return !waitForPosition || Util.epsilonEquals(getPosition(), angle, 1.0);
            }
        };
    }

    public Request wristAboveAngleWait(double angle) {
        return new Request() {
            @Override 
            public void act() {

            }

            @Override 
            public boolean isFinished() {
                return getPosition() > angle;
            }
        };
    }

    public Request setSlowRequest(boolean enable) {
        return new Request() {
            @Override
            public void act() {
                setWantSlow(enable);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    private void setWantSlow(boolean slow) {
        if (slow) {
            mMaster.configMotionCruiseVelocity(mConstants.kCruiseVelocity / 2, Constants.kCANTimeoutMs);
            mMaster.configMotionAcceleration(mConstants.kAcceleration / 2, Constants.kCANTimeoutMs);
        } else {
            mMaster.configMotionCruiseVelocity(mConstants.kCruiseVelocity, Constants.kCANTimeoutMs);
            mMaster.configMotionAcceleration(mConstants.kAcceleration, Constants.kCANTimeoutMs);
        }
    }

    @Override
    public boolean atHomingLocation() {
        return mPeriodicIO.position_units > mConstants.kHomePosition
                || Util.epsilonEquals(mPeriodicIO.position_units, mConstants.kHomePosition, 10.0);
    }

    @Log
    public double getWristAngleUnits() {
        return mPeriodicIO.position_units;
    }

    @Log
    public double getWristAngleTicks() {
        return mPeriodicIO.position_ticks;
    }

    @Log
    public double getWristSetpoints() {
        return getSetpointHomed();
    }

    @Log
    public double getWristDemand() {
        return mPeriodicIO.demand;
    }

    @Log
    public double getWristVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms;
    }

    @Log
    public double getWristVolts() {
        return mPeriodicIO.output_voltage;
    }

    @Log
    public double getWristCurrent() {
        return mPeriodicIO.master_current;
    }

    @Log
    public boolean getWristHoming() {
        return mHoming;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    @Log
    public double getMainMotorBusVolts() {
        return mMaster.getBusVoltage();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + " Angle Units", mPeriodicIO.position_units);
        SmartDashboard.putNumber(mConstants.kName + " Angle Ticks", mPeriodicIO.position_ticks);
        SmartDashboard.putNumber(mConstants.kName + " Demand", getSetpointHomed());
        SmartDashboard.putNumber(mConstants.kName + " Velocity", mPeriodicIO.velocity_ticks_per_100ms);
        SmartDashboard.putNumber(mConstants.kName + " Trajectory Velocity", mPeriodicIO.active_trajectory_velocity);
        SmartDashboard.putNumber(mConstants.kName + " Output Volts", mPeriodicIO.output_voltage);
        SmartDashboard.putNumber(mConstants.kName + " Stator Current", mPeriodicIO.master_current);
        SmartDashboard.putBoolean(mConstants.kName + " Homing", mHoming);
    }

}

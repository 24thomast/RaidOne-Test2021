package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Drive{
    private static TalonSRX talon0;
    private static TalonSRX talon1;
    private static TalonSRX talon2;
    private static TalonSRX talon3;
    private static TalonSRX talon4;
    private static TalonSRX talon5;

    public Drive(int tal0, int tal1, int tal2, int tal3, int tal4, int tal5){
        talon0 = new TalonSRX(tal0);
        talon1 = new TalonSRX(tal1);
        talon2 = new TalonSRX(tal2);
        talon3 = new TalonSRX(tal3);
        talon4 = new TalonSRX(tal4);
        talon5 = new TalonSRX(tal5);

        talon0.configFactoryDefault();
        talon1.configFactoryDefault();
        talon2.configFactoryDefault();
        talon3.configFactoryDefault();
        talon4.configFactoryDefault();
        talon5.configFactoryDefault();

        talon0.setNeutralMode(NeutralMode.Brake);
        talon1.setNeutralMode(NeutralMode.Brake);
        talon2.setNeutralMode(NeutralMode.Brake);
        talon3.setNeutralMode(NeutralMode.Brake);
        talon4.setNeutralMode(NeutralMode.Brake);
        talon5.setNeutralMode(NeutralMode.Brake);

		talon0.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		talon0.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		talon0.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon0.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon0.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon0.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		talon0.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		talon0.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		talon0.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		talon0.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		talon0.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		talon0.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);
		talon0.configMotionAcceleration(4000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		talon0.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        
        talon2.follow(talon0);
        talon4.follow(talon0);
        talon1.follow(talon0);
        talon3.follow(talon0);
        talon5.follow(talon0);

        talon1.setInverted(true);
        talon3.setInverted(true);
        talon5.setInverted(true);
    }

    public double motorOutput(){
        return talon0.getMotorOutputPercent();
    }

    public void run(ControlMode mode, double val){
        talon0.set(mode, val);
        System.out.println(mode);
    }

    public void setPos(int val){
        talon0.setSelectedSensorPosition(val);
    }

    public void configMotionSCurveStrength(int smoothing){
        talon0.configMotionSCurveStrength(smoothing);
    }

    public void setSelectedSensorPosition(int num){
        talon0.setSelectedSensorPosition(num);
    }
}


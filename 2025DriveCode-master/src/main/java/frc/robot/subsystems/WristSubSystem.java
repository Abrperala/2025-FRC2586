package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubSystem extends SubsystemBase {
    
    private final TalonFX wristMotor = new TalonFX(9);//TODO: update with correct value
    private final TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public enum WristPosition {//TODO: set these values correctly
        HOME(0), //encoder position 0 is park, 14 is algae
        ALGAEPICKUP(14),
        ALGAESHOOT(0);
        private double value;    
      
        private WristPosition(double value) {
          this.value = value;
        }
      
        public double getValue() {
          return value;
        }
    }
    public WristSubSystem(){
        wristConfig.Slot0.kP = 0.1;
        wristConfig.Slot0.kI = 0.0;
        wristConfig.Slot0.kD = 0.0;
        wristMotor.getConfigurator().apply(wristConfig);
    }

    public void setWristPosition(WristPosition state) {
        wristMotor.setControl(m_request.withPosition(state.getValue()).withVelocity(.1));
    }
    //NOTE: this is here for testing purposes primarily
    public void setSpeed(double speed) {
        SmartDashboard.putNumber("Wrist Encoder Position", getEncoderPosition());
        wristMotor.set(speed);
    }
    public double getEncoderPosition() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    // public void setPivot(double set) {
    //   if (isinRange()) {
    //       wristMotor.set(set);
    //   } else {
    //       System.out.println("Top Pivot Bad");
    //   }

    // public boolean isinRange() {
    //   if (getEncoderPosition() > Constants.MAX_TOP_PIVOT_ANGLE || getEncoderPosition() < Constants.MIN_TOP_PIVOT_ANGLE) {
    //     return false;
    //   } else {
    //       return true;
    //       }
     }



    /*
    private WristPosition currentWristPosition = WristPosition.LOW;
    public void wristUp(){
      switch(currentWristPosition){
        case LOW:
          currentWristPosition = WristPosition.MIDDLE;
          setWristPosition(currentWristPosition);
          break;
        case MIDDLE:
          currentWristPosition = WristPosition.HIGH;
          setWristPosition(currentWristPosition);
          break;
        case HIGH:
          break;

      }
    }
    public void wristDown(){
      switch(currentWristPosition){
        case LOW:
          break;
        case MIDDLE:
          currentWristPosition = WristPosition.LOW;
          setWristPosition(currentWristPosition);
          break;
        case HIGH:
          currentWristPosition = WristPosition.MIDDLE;
          setWristPosition(currentWristPosition);
          break;

      }
    } */


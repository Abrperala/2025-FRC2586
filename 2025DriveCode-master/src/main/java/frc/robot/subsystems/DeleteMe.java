package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class DeleteMe {
    private final DigitalInput hallEffectSwitch = new DigitalInput(2);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public static void main(String[] args) {
        DeleteMe deleteMe = new DeleteMe();
        deleteMe.checkConditions();
    }

    public void checkConditions() {
        double encoderPosition = elevatorSubsystem.getEncoder1Position();
        
        boolean isHallEffectSwitchPressed = hallEffectSwitch.get();

        if (encoderPosition > 100 && isHallEffectSwitchPressed) {
            System.out.println("Condition met: Encoder1 position is greater than 100 and Hall Effect Switch is pressed.");
            // Add your conditional logic here
        } else {
            System.out.println("Condition not met.");
        }
    }
}

// This class is a simple example of how to use the ElevatorSubsystem and DigitalInput classes to check conditions and execute logic based on those conditions.
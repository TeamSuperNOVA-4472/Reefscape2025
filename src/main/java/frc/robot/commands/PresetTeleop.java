package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/*
ALL TASKS THAT GOTTA BE DONE (Copied & Pasted From Kyle's Essay ;-;):
Task 1: The A button moves to the L1 position. (DONE/NOT-DONE)
Task 2: The X button moves to the L2 position. (DONE/NOT-DONE)
Task 3: The B button moves to the L3 position. (DONE/NOT-DONE)
Task 4: The Y button moves to the L4 position. (DONE/NOT-DONE)
Task 5: If no buttons are pressed, move to the stow position. (DONE/NOT-DONE)
Task 6: If the right bumper is pressed, move to the coral loading position and begin intaking coral once it reaches that position. (DONE/NOT-DONE)
Task 7: If the right trigger is pressed, begin outtaking coral. Stop outtaking once the button has been let go. (DONE/NOT-DONE)
Task 8: If the left bumper is pressed, move to the L2 Algae position and begin intaking algae once it reaches that position. (DONE/NOT-DONE)
Task 9: If the left trigger is pressed, move to the L3 Algae position and begin intaking. (DONE/NOT-DONE)
Task 10: If the dpad POV is at 90 (aka the right DPAD button is pressed), move to the algae ground position and begin intaking. (DONE/NOT-DONE)
Task 11: Stop intaking whenever the buttons are let go. (DONE/NOT-DONE)
Task 12: If the POV is at 180 (aka the bottom button is pressed), move to the algae processor position. Do not begin outtaking. (DONE/NOT-DONE)
Task 13: If the POV is at 270 (aka the left button), move to the algae barge position. Do not begin outtalking. (DONE/NOT-DONE)
Task 14: If the POV is at 0 (aka the up button), begin outtaking algae. Stop once it has been let go. (DONE/NOT-DONE)
Task 15: If all conditions fail, move back to the stow position. (DONE/NOT-DONE)
Task 16: If at any point you attempt to stow the carriage, check if algae is preset in the intake (IntakeSubsystem.hasAlgae() might be useful here). If it is, move to the algae stow position. (DONE/NOT-DONE)
*/

public class PresetTeleop extends Command
{
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;

    private final IntakeSubsystem mIntakeSubsystem;

    private final XboxController mController;

    private final Trigger aButton;

    private final Trigger xButton;

    private final Trigger bButton;

    private final Trigger yButton;

    private final Trigger rightBumper;

    private final Trigger rightTrigger;

    private final Trigger leftBumper;

    private final Trigger leftTrigger;

    private final Trigger dpadRight;

    private final Trigger dpadDown;

    private final Trigger dpadLeft;
    
    private final Trigger dpadUp;

    public PresetTeleop(ElevatorCarriageSubsystem elevatorCarriage, IntakeSubsystem intake, XboxController controller)
    {
        mElevatorCarriageSubsystem = elevatorCarriage;

        mIntakeSubsystem = intake;

        mController = controller;

        aButton = new Trigger(mController::getAButton);

        xButton = new Trigger(mController::getXButton);

        bButton = new Trigger(mController::getBButton);

        yButton = new Trigger(mController::getYButton);

        rightBumper = new Trigger(mController::getRightBumper);

        rightTrigger = new Trigger(() -> mController.getRightTriggerAxis() > 0.1);

        leftBumper = new Trigger(mController::getLeftBumper);

        leftTrigger = new Trigger(() -> mController.getLeftTriggerAxis() > 0.1);

        dpadRight = new Trigger(() -> mController.getPOV() == 90);    

        dpadDown = new Trigger(() -> mController.getPOV() == 180);

        dpadLeft = new Trigger(() -> mController.getPOV() == 270);

        dpadUp = new Trigger(() -> mController.getPOV() == 0);    

        addRequirements(mElevatorCarriageSubsystem);
    }

    @Override
    public void execute()
    {
        CarriagePreset target = CarriagePreset.kStowCoral;

        // Task 1
        if (aButton.getAsBoolean())
        {
            target = CarriagePreset.kCoralL1;
        }

        // Task 2
        else if (xButton.getAsBoolean())
        {
            target = CarriagePreset.kCoralL2;
        }

        // Task 3
        else if (bButton.getAsBoolean())
        {
            target = CarriagePreset.kCoralL3;
        }

        // Task 4
        else if (yButton.getAsBoolean())
        {
            target = CarriagePreset.kCoralL4;
        }

        // Task 6
        else if (rightBumper.getAsBoolean())
        {
            target = CarriagePreset.kCoralLoad;

            if (mElevatorCarriageSubsystem.isAtPosition(target))
            {
                mIntakeSubsystem.intakeCoral();
            }
        }

        // Task 7
        else if (rightTrigger.getAsBoolean())
        {
            mIntakeSubsystem.outtakeCoral();
        }

        // Task 11
        else
        {
            mIntakeSubsystem.stop();
        }

        // Task 8
        if (leftBumper.getAsBoolean())
        {
            target = CarriagePreset.kAlgaeL2;

            if (mElevatorCarriageSubsystem.isAtPosition(target))
            {
                mIntakeSubsystem.intakeAlgae();
            }
        }

        // Task 9
        else if (leftTrigger.getAsBoolean())
        {
            target = CarriagePreset.kAlgaeL3;

            if (mElevatorCarriageSubsystem.isAtPosition(target))
            {
                mIntakeSubsystem.intakeAlgae();
            }
        }

        // Task 10
        else if (dpadRight.getAsBoolean())
        {
            target = CarriagePreset.kAlgaeGround;

            if (mElevatorCarriageSubsystem.isAtPosition(target))
            {
                mIntakeSubsystem.intakeAlgae();
            }
        }

        // Task 11
        if (!mController.getLeftBumper() && mController.getLeftTriggerAxis() <= 0.1 && mController.getPOV() != 90)
        {
            mIntakeSubsystem.stop();
        }

        // Task 12
        else if (dpadDown.getAsBoolean())
        {
            target = CarriagePreset.kAlgaeProcessor;
        }

        // Task 13
        else if (dpadLeft.getAsBoolean())
        {
            target = CarriagePreset.kAlgaeBarge;
        }

        // Task 14
        else if (dpadUp.getAsBoolean())
        {
            mIntakeSubsystem.outtakeAlgae();
        }

        // Task 11
        else
        {
            mIntakeSubsystem.stop();
        }

        // Task 16
        if (target == CarriagePreset.kStowCoral && mIntakeSubsystem.hasAlgae())
        {
            target = CarriagePreset.kStowAlgae;
        }
    }

    // added just for the safety during situations like climbing and such, to stop everything
    @Override
    public void end(boolean stopping)
    {
        mElevatorCarriageSubsystem.stop();

        mIntakeSubsystem.stop();
    }
}

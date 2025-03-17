package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class PresetTeleop
{
    public static void setup(XboxController partner)
    {
        // Set up partner controls with triggers.
        ElevatorCarriageSubsystem elevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        IntakeSubsystem intake = IntakeSubsystem.kInstance;

        // If nothing is to be done, stow the carriage.
        elevatorCarriage.setDefaultCommand(SwitchPresetCommand.stow(true));

        // #region Controls
        Trigger moveL1 = new Trigger(partner::getAButton),
                moveL2 = new Trigger(partner::getXButton),
                moveL3 = new Trigger(partner::getBButton),
                moveL4 = new Trigger(partner::getYButton);

        Trigger loadCoral = new Trigger(partner::getRightBumperButton),
                scoreCoral = new Trigger(() -> partner.getRightTriggerAxis() > 0.25);

        Trigger algaeL2 = new Trigger(partner::getLeftBumperButton),
                algaeL3 = new Trigger(() -> partner.getLeftTriggerAxis() > 0.25),
                algaeGround = new Trigger(() -> partner.getPOV() == 90);

        Trigger algaeProccessor = new Trigger(() -> partner.getPOV() == 180),
                algaeBarge = new Trigger(() -> partner.getPOV() == 270);

        Trigger algaeOutake = new Trigger(() -> partner.getPOV() == 0);
        // #endregion

        // #region Check for moving to L1, L2, L3, and L4 positions.
        moveL1.whileTrue(new SwitchPresetCommand(CarriagePreset.kCoralL1, true));
        moveL2.whileTrue(new SwitchPresetCommand(CarriagePreset.kCoralL2, true));
        moveL3.whileTrue(new SwitchPresetCommand(CarriagePreset.kCoralL3, true));
        moveL4.whileTrue(new SwitchPresetCommand(CarriagePreset.kCoralL4, true));
        // #endregion

        // #region Handle loading and scoring coral.
        loadCoral.whileTrue(new SequentialCommandGroup(
            SwitchPresetCommand.load(false),
            new InstantCommand(() -> intake.intakeCoral()),
            new ForeverCommand()
        ));
        loadCoral.onFalse(new InstantCommand(() -> intake.stopCoral()));

        scoreCoral.onTrue(new InstantCommand(() -> intake.outtakeCoral()));
        scoreCoral.onFalse(new InstantCommand(() -> intake.stopCoral()));
        // #endregion

        // #region Go to Algae L2, L3, and ground, begin intaking.
        algaeL2.whileTrue(new SequentialCommandGroup(
            new SwitchPresetCommand(CarriagePreset.kAlgaeL2, false),
            new InstantCommand(() -> intake.intakeAlgae()),
            new ForeverCommand()
        ));
        algaeL2.onFalse(new InstantCommand(() -> intake.stopAlgae()));

        algaeL3.whileTrue(new SequentialCommandGroup(
            new SwitchPresetCommand(CarriagePreset.kAlgaeL3, false),
            new InstantCommand(() -> intake.intakeAlgae()),
            new ForeverCommand()
        ));
        algaeL3.onFalse(new InstantCommand(() -> intake.stopAlgae()));
        
        algaeGround.whileTrue(new SequentialCommandGroup(
            new SwitchPresetCommand(CarriagePreset.kAlgaeGround, false),
            new InstantCommand(() -> intake.intakeAlgae()),
            new ForeverCommand()
        ));
        algaeGround.onFalse(new InstantCommand(() -> intake.stopAlgae()));
        // #endregion

        // #region Algae Processor and Barge
        algaeProccessor.whileTrue(new SwitchPresetCommand(CarriagePreset.kAlgaeProcessor, true));
        algaeBarge.whileTrue(new SwitchPresetCommand(CarriagePreset.kAlgaeBarge, true));
        // #endregion

        // #region Outtake Algae
        algaeOutake.onTrue(new InstantCommand(() -> intake.outtakeAlgae()));
        algaeOutake.onFalse(new InstantCommand(() -> intake.stopAlgae()));
        // #endregion
    }
}

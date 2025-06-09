package org.firstinspires.ftc.teamcode.example.java.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
@TeleOp
public class SubsystemsTest extends NextFTCOpMode {
    public SubsystemsTest() {
        super(Lift.INSTANCE);
    }

    @Override
    public void onInit() {
        // Initialize subsystems here if needed

    }
    @Override
    public void onStartButtonPressed() {
        // This method can be used to start commands or set initial states
        // For example, you could invoke a com
        // mand to move the lift to a specific position
        Lift.INSTANCE.toHighBasket().invoke();
        gamepadManager.getGamepad2().getCircle().setPressedCommand(Lift.INSTANCE::retract);
    }
    @Override
    public void onUpdate() {
        // Update logic for the test here
        // This could include invoking commands or checking subsystem states

    }
}

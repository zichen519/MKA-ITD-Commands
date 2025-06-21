package org.firstinspires.ftc.teamcode.example.java.commands;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

public class EndEffectorPositions {

    // Preset position commands using multiple subsystems
    public static Command basketScore() {
        return new ParallelGroup(
                Elbow.INSTANCE.setPosition(0.55),     // Elbow up for basket height
                Wrist.INSTANCE.setPosition(0.2),     // Wrist angled for basket
                Rotate.INSTANCE.setPosition(0.52)   // Neutral rotation
        );
    }

    public static Command avoidBasket(){
        return Elbow.INSTANCE.setPosition(.42);
    }

    public static Command grabFromWall() {
        return new ParallelGroup(

                Elbow.INSTANCE.setPosition(0.99),     // Elbow positioned for wall height
                Wrist.INSTANCE.setPosition(0.65),     // Wrist angled for wall grab
                Rotate.INSTANCE.setPosition(0.8)   // Neutral rotation
        );
    }

    public static Command ramScore(){
        return new SequentialGroup(
                Wrist.INSTANCE.setPosition(1),
                new Delay(0.5),
                new ParallelGroup(
                        Wrist.INSTANCE.setPosition(.25),
                        Elbow.INSTANCE.setPosition(.45),
                        Rotate.INSTANCE.setPosition(.23),
                        Linkage.INSTANCE.setTarget(-850)
                )

        );
    }

    public static Command preChamberScore() {
        return new SequentialGroup(
                // First movement - position arm for chamber approach
                Wrist.INSTANCE.setPosition(1),
                // Half second delay
                new Delay(0.5),
                // Second movement - fine positioning
                new ParallelGroup(
                        Elbow.INSTANCE.setPosition(0.5),     // Elbow at mid position
                        Wrist.INSTANCE.setPosition(0.3),     // Wrist prepped for chamber approach
                        Rotate.INSTANCE.setPosition(0.5)       // Wrist prepped for chamber approach
                )
        );
    }

    public static Command scoreOnChamber() {
        return Elbow.INSTANCE.setPosition(0.12);
    }

    public static Command hoverAboveFloor() {
        return new ParallelGroup(
                Elbow.INSTANCE.setPosition(0.575),     // Elbow low but not touching floor
                Wrist.INSTANCE.setPosition(.85),
                Claw.INSTANCE.open()// Wrist angled down toward floor

        );
    }

    public static Command grabFromFloor() {
        return new SequentialGroup(
                Claw.INSTANCE.open(),
                new Delay(0.2), // Wait for claw to open
                new ParallelGroup(
                        Elbow.INSTANCE.setPosition(0.45),     // Elbow fully down to floor level
                        Wrist.INSTANCE.setPosition(.85)
                ),
                new Delay(0.2), // Wait for elbow to settle
                Claw.INSTANCE.close(),
                new Delay(0.2),
                Elbow.INSTANCE.setPosition(0.575)
        );

    }






}

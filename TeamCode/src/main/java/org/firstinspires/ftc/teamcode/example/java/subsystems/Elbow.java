package org.firstinspires.ftc.teamcode.example.java.subsystems;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;

import java.util.List;
import java.util.Set;

// Elbow subsystem (dual servo)
public class Elbow extends Subsystem {
    public static final Elbow INSTANCE = new Elbow();
    private Elbow() { }

    public Servo elbow1;
    public Servo elbow2;

    public String elbowLeft = "elbow1";
    public String elbowRight = "elbow2";

    public Command setPosition(double position) {
        return new MultipleServosToPosition(
                List.of(elbow1, elbow2),
                position,
                this
        );
    }

    @Override
    public void initialize() {
        elbow1 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, elbowLeft);
        elbow2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, elbowRight);
        elbow1.setDirection(Servo.Direction.REVERSE); // Ensure both servos move in sync
    }
}

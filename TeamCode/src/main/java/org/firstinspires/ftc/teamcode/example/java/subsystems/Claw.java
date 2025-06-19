package org.firstinspires.ftc.teamcode.example.java.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
public class Claw extends Subsystem {
    public static final Claw INSTANCE = new Claw();
    private Claw() { }

    public Servo grab;
    public String clawName = "grab";

    public Command open() {
        return new ServoToPosition(grab, .17, this);
    }

    public Command close() {
        return new ServoToPosition(grab, 0.6, this);
    }

    public Command setPosition(double position) {
        return new ServoToPosition(grab, position, this);
    }

    @Override
    public void initialize() {
        grab = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawName);
    }
}
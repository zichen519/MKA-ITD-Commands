package org.firstinspires.ftc.teamcode.example.java.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
public class Rotate extends Subsystem {
    public static final Rotate INSTANCE = new Rotate();
    private Rotate() { }
    public double offset = 0.2;
    public Servo rotate;
    public String rotateName = "rotate";

    public Command setPosition(double position) {
        return new ServoToPosition(rotate, position, this);
    }

    @Override
    public void initialize() {
        rotate = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, rotateName);
    }
}
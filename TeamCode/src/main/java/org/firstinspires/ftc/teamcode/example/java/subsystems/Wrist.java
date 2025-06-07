package org.firstinspires.ftc.teamcode.example.java.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
public class Wrist extends Subsystem {
    public static final Wrist INSTANCE = new Wrist();
    private Wrist() { }

    public Servo wrist;
    public String wristName = "wrist";

    public Command setPosition(double position) {
        return new ServoToPosition(wrist, position, this);
    }

    @Override
    public void initialize() {
        wrist = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, wristName);
    }
}
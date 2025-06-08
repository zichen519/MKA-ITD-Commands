package org.firstinspires.ftc.teamcode.example.java.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.example.java.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;

@TeleOp
public class LimelightTesting extends NextFTCOpMode {
    public LimelightTesting() {
        super(Rotate.INSTANCE, Limelight.INSTANCE);
    }
    private Limelight3A limelight;
    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
    @Override
    public void onStartButtonPressed() {
        Rotate.INSTANCE.setPosition((Limelight.INSTANCE.angle/255) + Rotate.INSTANCE.offset).invoke();
    }
}
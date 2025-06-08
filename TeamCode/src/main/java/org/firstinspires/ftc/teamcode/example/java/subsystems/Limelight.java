package org.firstinspires.ftc.teamcode.example.java.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

public class Limelight extends Subsystem {
    public static Limelight INSTANCE = new Limelight();

    private Limelight() {
    }

    public Limelight3A ll;
    public String name = "limelight";
    public LLResult result;

    public double angle;

    private Command getSampleAngle() {
        return new LambdaCommand()
                .setUpdate(() -> {
                    // Replace this with however you get the target pose.
                    // If the limelight output is null set samplePose to null,
                    // Otherwise set it to the pose
                    // You can have many lines of code here if needed.

                    ll.start();
                    result = ll.getLatestResult();
                    if (result== null) {
                        angle = -1;
                    }else{
                        double[] python = result.getPythonOutput();

                        angle = python[5];

                        ll.pause();
                    }
                })
                .setIsDone(() -> angle >=0);
    }



    @Override
    public void initialize(){
        ll = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, name);
    }
    public void periodic(){
        OpModeData.telemetry.addData("Limelight Angle", angle);
        OpModeData.telemetry.addData("valid", result != null);
        OpModeData.telemetry.update();
    }
}
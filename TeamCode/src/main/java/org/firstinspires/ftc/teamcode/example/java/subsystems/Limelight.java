package org.firstinspires.ftc.teamcode.example.java.subsystems;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroData;

public class Limelight extends Subsystem {
    public static Limelight INSTANCE = new Limelight();

    private Limelight() {
    }

    public Limelight3A limelight;
    public String name = "limelight";


    // Detection results

    public Command alignToBlock() {
        return new SequentialGroup(
                // Get limelight data and create path
                new LambdaCommand()
                        .setStart(() -> {
                            limelight.start();
                            LLResult result = limelight.getLatestResult();

                            if (result != null) {
                                double[] python = result.getPythonOutput();

                                // Debug: Print all python values
                                OpModeData.telemetry.addData("Python Array Length", python.length);
                                for (int i = 0; i < python.length && i < 10; i++) {
                                    OpModeData.telemetry.addData("python[" + i + "]", "%.3f", python[i]);
                                }

                                if (python.length >= 9 && python[0] == 1.0) {
                                    // Get offsets in inches from pipeline
                                    double xOffsetInches = python[7];  // Forward/backward
                                    double yOffsetInches = python[8];  // Left/right
                                    double distance = python[6];       // Distance
                                    double angle = python[5];          // Angle

                                    OpModeData.telemetry.addData("Block Detected", "YES");
                                    OpModeData.telemetry.addData("X Offset", "%.3f inches", xOffsetInches);
                                    OpModeData.telemetry.addData("Y Offset", "%.3f inches", yOffsetInches);
                                    OpModeData.telemetry.addData("Distance", "%.3f inches", distance);
                                    OpModeData.telemetry.addData("Angle", "%.1f degrees", angle);

                                    // Only move if offsets are reasonable (not the constant 7, 0)
                                    if (Math.abs(xOffsetInches) < 5 && Math.abs(yOffsetInches) < 5) {
                                        // Calculate target pose
                                        Pose currentPose = PedroData.INSTANCE.getFollower().getPose();
                                        double robotHeading = Math.toRadians(-90); // Always -90 degrees

                                        double fieldX = currentPose.getX() +
                                                xOffsetInches * Math.cos(robotHeading) -
                                                yOffsetInches * Math.sin(robotHeading);
                                        double fieldY = currentPose.getY() +
                                                xOffsetInches * Math.sin(robotHeading) +
                                                yOffsetInches * Math.cos(robotHeading);

                                        Pose targetPose = new Pose(fieldX, fieldY, robotHeading);

                                        // Create and follow path
                                        new FollowPath(
                                                new PathBuilder()
                                                        .addPath(
                                                                new BezierLine(
                                                                        currentPose,
                                                                        targetPose
                                                                )
                                                        )
                                                        .build()
                                        ).invoke();

                                        OpModeData.telemetry.addData("Action", "Moving to target");
                                    } else {
                                        OpModeData.telemetry.addData("Action", "Offsets seem wrong - not moving");
                                    }
                                } else {
                                    OpModeData.telemetry.addData("Status", "No block detected or wrong array format");
                                }
                            } else {
                                OpModeData.telemetry.addData("Status", "No limelight data");
                            }

                            limelight.pause();
                            OpModeData.telemetry.update();
                        })
                        .setIsDone(() -> !PedroData.INSTANCE.getFollower().isBusy())
        );
    }



    @Override
    public void initialize(){
        limelight = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, name);
        limelight.setPollRateHz(10);
        limelight.pipelineSwitch(3);
    }
    public void periodic(){


    }
}
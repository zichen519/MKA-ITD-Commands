package org.firstinspires.ftc.teamcode.example.java.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.example.java.commands.EndEffectorPositions;
import org.firstinspires.ftc.teamcode.example.java.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.example.java.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@TeleOp
public class LimelightTesting extends PedroOpMode {
    public LimelightTesting() {
        super(
                Rotate.INSTANCE,
                Elbow.INSTANCE,
                Wrist.INSTANCE,
                Claw.INSTANCE
        );
    }
    private Pose startPose = new Pose(0,0,Math.toRadians(-90));

    private boolean prevCircle;

    private Limelight3A limelight;
    @Override
    public void onInit() {
        OpModeData.telemetry = telemetry;
        EndEffectorPositions.hoverAboveFloor().invoke();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        limelight = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.pipelineSwitch(3);

    }

    @Override
    public void onStartButtonPressed() {
        gamepadManager.getGamepad2().getCross().setPressedCommand(
                () -> EndEffectorPositions.grabFromFloor()
        );
        limelight.start();
    }

    @Override
    public void onUpdate() {
        boolean currCircle = gamepad2.circle;
        LLResult result = limelight.getLatestResult();
        if (result != null && currCircle && !prevCircle) {

            double[] outputs = result.getPythonOutput();
            double xOffsetInches = outputs[7];  // Forward/backward
            double yOffsetInches = outputs[8];
            double robotHeading = Math.toRadians(-90);
            Pose currentPose = follower.getPose();
            double fieldX = currentPose.getX() - yOffsetInches;  // Y offset affects X movement (left/right)
            double fieldY = currentPose.getY() - xOffsetInches;  // X offset affects Y movement (forward/back)
            telemetry.addData("x", xOffsetInches);
            telemetry.addData("y", yOffsetInches);
            Pose targetPose = new Pose(fieldX, fieldY, robotHeading);
            new FollowPath(
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Point(currentPose), new Point(targetPose)
                                    )
                            ).setLinearHeadingInterpolation(currentPose.getHeading(),targetPose.getHeading())
                            .build()
                    , true
            ).invoke();
            Rotate.INSTANCE.setPosition((outputs[5]/255)+0.2).invoke();
        }
        prevCircle = currCircle;


        telemetry.update();

    }
}
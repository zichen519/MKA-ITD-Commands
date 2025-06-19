package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.example.java.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.example.java.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

public class SpecimenAuto extends PedroOpMode {
    public SpecimenAuto(){
        super(
                Claw.INSTANCE,
                Elbow.INSTANCE,
                Lift.INSTANCE,
                Linkage.INSTANCE,
                Rotate.INSTANCE,
                Wrist.INSTANCE
        );
    }

    private final Pose startPose = new Pose(6.5,66, Math.toRadians(0.0));
    private final Pose scorePose1 = new Pose(39, 74, Math.toRadians(0));
    private final Pose pushPose1 = new Pose(57, 23, Math.toRadians(0));
    private final Pose humanPose1 = new Pose(31, 23, Math.toRadians(0));
    private final Pose pushPose2 = new Pose(49.000, 13.500, Math.toRadians(0));
    private final Pose humanPose2 = new Pose(31, 13.500, Math.toRadians(0));
    private final Pose humanPose3 = new Pose(20, 7.5, Math.toRadians(0));

    private PathChain score1, push1, human1, push2, human2, human3;

    public void buildPaths(){
        score1 = follower.pathBuilder().addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        push1 = follower.pathBuilder().addPath(new BezierLine(new Point(scorePose1), new Point(pushPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        human1 = follower.pathBuilder().addPath(new BezierLine(new Point(pushPose1), new Point(humanPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        push2 = follower.pathBuilder().addPath(new BezierLine(new Point(humanPose1), new Point(pushPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        human2 = follower.pathBuilder().addPath(new BezierLine(new Point(pushPose2), new Point(humanPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        human3 = follower.pathBuilder().addPath(new BezierLine(new Point(humanPose2), new Point(humanPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
    }

    public Command specAuto(){
        return new SequentialGroup(

        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OpModeData.telemetry = telemetry;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        Claw.INSTANCE.close().invoke();
        OpModeData.telemetry.addData("Status", "Initialized");
        OpModeData.telemetry.update();
    }

    @Override
    public void onStartButtonPressed(){
        specAuto().invoke();

    }
}

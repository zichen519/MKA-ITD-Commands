package org.firstinspires.ftc.teamcode.example.java.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadManager;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@TeleOp
@Config
public class PositionsTester extends NextFTCOpMode {

    public PositionsTester() {
        super(
                Elbow.INSTANCE,
                Wrist.INSTANCE,
                Rotate.INSTANCE,
                Claw.INSTANCE,
                Lift.INSTANCE,
                Linkage.INSTANCE

        );
    }

    // Dashboard configurable values
    public static double elbow = 0.0;  // Cosine feedforward constant
    public static double wrist = 0.0;
    public static double rotate = 0.0;
    public static double claw = 0.0;
    public static int liftTarget = 0;
    public static int linkageTarget = 0;



    @Override
    public void onInit() {
        // Initialize motor
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OpModeData.telemetry = telemetry;





    }
    @Override
    public void onStartButtonPressed(){
        gamepadManager.getGamepad2().getCircle().setPressedCommand(

                () -> new ParallelGroup(
                        Linkage.INSTANCE.setTarget(linkageTarget),
                        Lift.INSTANCE.setTarget(liftTarget),
                        Elbow.INSTANCE.setPosition(elbow),
                        Wrist.INSTANCE.setPosition(wrist),
                        Rotate.INSTANCE.setPosition(rotate),
                        Claw.INSTANCE.setPosition(claw)
                )
        );
    }


    @Override
    public void onUpdate() {
        // Update controller with dashboard values




    }


}
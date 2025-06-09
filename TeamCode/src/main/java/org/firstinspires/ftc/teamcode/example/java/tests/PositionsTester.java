package org.firstinspires.ftc.teamcode.example.java.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@TeleOp(name = "Linkage Tuning")
@Config
public class PositionsTester extends NextFTCOpMode {

    public PositionsTester() {
        super(
                Elbow.INSTANCE,
                Wrist.INSTANCE,
                Rotate.INSTANCE,
                Claw.INSTANCE,
                Lift.INSTANCE,
                Linkage.INSTANCE,
                Limelight.INSTANCE
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
        Linkage.INSTANCE.setTarget(linkageTarget).invoke();
        Lift.INSTANCE.toHighBasket().invoke();
        Elbow.INSTANCE.setPosition(elbow).invoke();
        Wrist.INSTANCE.setPosition(wrist).invoke();
        Rotate.INSTANCE.setPosition(rotate).invoke();
        Claw.INSTANCE.setPosition(claw).invoke();
    }


    @Override
    public void onUpdate() {
        // Update controller with dashboard values




    }


}
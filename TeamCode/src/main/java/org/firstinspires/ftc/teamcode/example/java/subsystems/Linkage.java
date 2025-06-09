package org.firstinspires.ftc.teamcode.example.java.subsystems;


import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;

@Config

public class Linkage extends Subsystem {
    // BOILERPLATE
    public static final Linkage INSTANCE = new Linkage();
    private Linkage() { }

    // USER CODE
    public MotorEx linkageMotor;

    public static double ticksPerRevolution = 1993.6;  // GoBuilda Yellow Jacket 84 RPM
    public static double gearRatio = 2.5;

    // Dashboard tunable variables
    public static double p = 0.006;
    public static double i = 0.0;
    public static double d = 0.0002;

    public static int target = 0;

    private double calculateFeedforward() {
        return Math.cos(Math.toRadians(target / ((ticksPerRevolution * gearRatio)/360))) * 0.045;
    }

    // Arm feedforward (better for pivoting mechanisms than static feedforward)
    ;
    public PIDFController controller = new PIDFController(0.006, 0.0,0.0002, v -> calculateFeedforward(), 30);

    // Motor configuration
    public String motorName = "lift";

    // Position constants (in encoder ticks)





    public Command linkageUp() {
        return new RunToPosition(linkageMotor, (double)-50, controller, this);
    }

    public Command linkageDown() {

        return new RunToPosition(linkageMotor, (double)-1650, controller, this);
    }



    public Command setTarget(int newTarget) {
        target = newTarget;
        return new RunToPosition(linkageMotor, (double)newTarget, controller, this);
    }
    @Override
    public Command getDefaultCommand(){
        return new HoldPosition(linkageMotor,controller,this);
    }
    @Override
    public void initialize() {
        linkageMotor = new MotorEx(motorName);
        linkageMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Adjust as needed
        linkageMotor.resetEncoder();

          // Smaller tolerance for precise positioning
    }



    public void periodic() {
        // Update PID values from dashboard
        /*
        controller.setKP(p);
        controller.setKI(i);
        controller.setKD(d);
        linkageMotor.setPower(controller.calculate(linkageMotor.getCurrentPosition(),target));
        // Update arm feedforward from dashboard
        */

        double currentAngle = linkageMotor.getCurrentPosition() / (ticksPerRevolution * gearRatio) * 360;
        double targetAngle = target / (ticksPerRevolution * gearRatio) * 360;
        // Telemetry

        OpModeData.telemetry.addData("Linkage Current Position", "%.1f ticks", linkageMotor.getCurrentPosition());
        //OpModeData.telemetry.addData("Target Position", "%d ticks", target);
        OpModeData.telemetry.addData("Current Angle", "%.1f°", currentAngle);
        OpModeData.telemetry.addData("Target Angle", "%.1f°", targetAngle);
        OpModeData.telemetry.addData("Linkage Motor Power", "%.3f", linkageMotor.getPower());


        //OpModeData.telemetry.addData("PID Values", "P:%.3f I:%.3f D:%.3f", p, i, d);
        OpModeData.telemetry.update();
        // Position status



    }
}

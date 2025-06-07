package org.firstinspires.ftc.teamcode.example.java;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

public class Lift extends Subsystem {
    // BOILERPLATE
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE
    public MotorEx leftMotor;
    public MotorEx rightMotor;
    public MotorGroup liftMotors;
    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));



    public String leftMotorName = "slide1";
    public String rightMotorName = "slide2";

    public Command manualControl(float power) {
        return new SetPower(liftMotors, (double)power, this);
    }
    public Command retract() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLowBasket() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighBasket() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                500.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighChamber() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                1200.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    
    @Override
    public void initialize() {
        leftMotor = new MotorEx(leftMotorName);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = new MotorEx(rightMotorName);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotors = new MotorGroup(leftMotor, rightMotor);
        rightMotor.resetEncoder();
        leftMotor.resetEncoder();
        stop();
        controller.setSetPointTolerance(30);
    }

    public void stop() {
        liftMotors.setPower(0);
    }
}

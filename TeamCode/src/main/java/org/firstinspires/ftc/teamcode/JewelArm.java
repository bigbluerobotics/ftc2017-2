package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class JewelArm {

    private double armUp = 0.9;
    private double armDown = 0.3;
    private double hitForPos = 0.3;
    private double hitNeutralPos = 0.5;
    private double hitBackPos = 0.7;

    public Servo xServo;
    public Servo yServo;
    public ColorSensor colorSensor;


    public JewelArm(HardwareMap hardwareMap){
        xServo = hardwareMap.get(Servo.class, RobotMap.jewelServoX);
        yServo = hardwareMap.get(Servo.class, RobotMap.jewelServoY);
        colorSensor = hardwareMap.get(ColorSensor.class, "jewel_color_sensor");
    }

    public void lower(){
        yServo.setPosition(armDown);
        xServo.setPosition(hitNeutralPos);
    }

    public void hitJewel(boolean teamColorIsRed){
        boolean detectsRed = colorSensor.red() > (colorSensor.blue()+10);
        if(detectsRed == teamColorIsRed){
            hitBackward();
        }else{
            hitForward();
        }
    }

    public void hitBackward(){
        xServo.setPosition(hitBackPos);
    }

    public void hitForward(){
        xServo.setPosition(hitForPos);
    }

    public void raise(){
        xServo.setPosition(hitNeutralPos);
        yServo.setPosition(armUp);
    }

}

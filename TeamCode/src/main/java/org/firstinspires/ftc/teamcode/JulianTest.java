

package org.firstinspires.ftc.teamcode;

import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
@Autonomous(name="Julian Test")
public abstract class JulianTest extends LinearOpMode
{
    public static final boolean RED = true;
    public static final boolean BLUE = false;

    public ElapsedTime runtime = new ElapsedTime();
    public MecanumDrive mecanumDrive = null;
    public ColorSensor leftColor = null;
    public ColorSensor rightColor = null;
    public boolean allianceColor = BLUE;
    public char glyphPos = 'c';
    public int redThresh;//threshold between field color and red line
    public int blueThresh;//threshold between field color and blue line
    public double strafeDist = 10.0; //modify, distance you need to strafe to go from one column to an adjacent column

    public PingryAutonomous() {}

    public void runOpMode() throws InterruptedException{//assuming you start at center platform
    leftColor = hardwareMap.get(ColorSensor.class, "left_color_sensor");
    rightColor = hardwareMap.get(ColorSensor.class, "right_color_sensor");
    //sonarSensor = hardwareMap.get(ColorSensor.class, "sonar_sensor");
    //after first glyph is dunked
    if(glyphPos == 'l')//get robot centered
        mecanumDrive.moveStrafe(strafeDist,0.3);
    else if(glyphPos == 'r')
        mecanumDrive.moveStrafe(-strafeDist,0.3);
    mecanumDrive.moveEncoderStraight(5, .4);//move forward a bit so that you no longer see parking lines
    align(0.3);
    ///////////////////////////pick up glyphs, when this ends robot should be able to backup and be somewhat close (+- 8 in) to aligned with center of glyph pit line when it hits the glyph pit line///////////////////////////////////////////
    align(-0.3);
    mecanumDrive.moveEncoderStraight(5, .4);//move forward a bit so that you no longer see parking lines
    twait(800);
    while((this.allianceColor == RED && (leftColor.red() < redThresh || rightColor.red() < redThresh)) || (this.allianceColor == BLUE && (leftColor.blue() < blueThresh || rightColor.blue() < blueThresh)){//while you are not aligned to glyph line
        if((this.allianceColor == RED && leftColor.red() < redThresh && rightColor.red() > redThresh) || (this.allianceColor == BLUE && leftColor.blue() < blueThresh && rightColor.blue() > blueThresh)) //if only right color sensor sees line
            mecanumDrive.move(.3,-.6,-.6,.3)//strafe 45° right/back (southeast)
        else if((this.allianceColor == RED && leftColor.red() > redThresh && rightColor.red() < redThresh) || (this.allianceColor == BLUE && leftColor.blue() > blueThresh && rightColor.blue() < blueThresh))//if only left color sensor sees line
            mecanumDrive.move(-.6, .3, .3, -.6);//strafe 45° left/back (southwest)
        else
            mecanumDrive.move(-.3, -.3, -.3, -.3);
        twait(15);
    }
    mecanumDrive.moveEncoderStraight(-2, .3);
    if(glyphPos == 'c')//put glyphs in another column
        mecanumDrive.moveStrafe(strafeDist,0.3);
    this.dunkGlyph();
}

    public void align(double pow) throws InterruptedException {//go forward (positive pow) or backward and align to line
        while((this.allianceColor == RED && (leftColor.red() < redThresh || rightColor.red() < redThresh)) || (this.allianceColor == BLUE && (leftColor.blue() < blueThresh || rightColor.blue() < blueThresh)){//while you are not aligned to glyph line
            if((this.allianceColor == RED && leftColor.red() < redThresh && rightColor.red() > redThresh) || (this.allianceColor == BLUE && leftColor.blue() < blueThresh && rightColor.blue() > blueThresh)) //if only right color sensor sees line
                mecanumDrive.move(pow, -pow, pow, -pow);//turn right if pow > 0
            else if((this.allianceColor == RED && leftColor.red() > redThresh && rightColor.red() < redThresh) || (this.allianceColor == BLUE && leftColor.blue() > blueThresh && rightColor.blue() < blueThresh))//if only left color sensor sees line
                mecanumDrive.move(-pow,pow,-pow,pow)//turn left if pow > 0
            else
                mecanumDrive.move(pow, pow, pow, pow);
            twait(15);
        }
    }

    public void dunkGlyph() throws InterruptedException {

    }


    public void twait(long millis) throws InterruptedException{
        Thread.sleep(millis);
    }

}

*/



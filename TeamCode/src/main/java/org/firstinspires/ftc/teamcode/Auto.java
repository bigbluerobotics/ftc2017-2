package org.firstinspires.ftc.teamcode;

import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

public abstract class Auto extends LinearOpMode
{
    public static final boolean RED = true;
    public static final boolean BLUE = false;

    public ElapsedTime runtime = new ElapsedTime();
    public MecanumDrive mecanumDrive = null;
    public GlyphMechanism glyphMechanism = null;
    public JewelArm jewelArm = null;

    public boolean allianceColor = BLUE;

    public VuforiaLocalizer vuforia;
    public VuforiaTrackable relicTemplate;
    public char glyphPos = 'c';

    public Auto() {}

    public void runOpMode() throws InterruptedException{
        mecanumDrive = new MecanumDrive(hardwareMap);
        glyphMechanism = new GlyphMechanism(hardwareMap);
        jewelArm = new JewelArm(hardwareMap);

        glyphMechanism.grabBottom();

        /*Vuforia Code */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AUx/Jtr/////AAAAGT/Wv/qzqE8Io1FwGTLz740qlNzPptr0US0IHzHFiciwzXK4addvdjXKjbRTPQjctOoOqR7ePutjttVjdpN723q6dVpCqV5te9sGoybNn78dC7TnzNbCXjCPqgTlWxpDsKx/Dy45z8xBKjKKMmTtNSCszpMVGl7ggM5RjDmzPU8vZxbhZAEHDeDdhk5jvxgLOYw208Y+vpUMRcSSU+57D8h6YKTFscxsJXz/xAwAwONxdI3wmySFbX7y/WjheD5bqkbNse6Plz6a1RoFcdHdzsW1W67auC8IQ41kC5LlhibT/61kKlBhCxQxEuLpbzO1UAASvysZsIpBBexP1ZRMqYQynJ7Qd4CG0dTNBwEArxHF";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        relicTrackables.activate();

        waitForStart();

        runtime.reset();
        detectPattern();
        glyphMechanism.grabBottom();

        //glyphMechanism.flipDown();
        this.hitJewel();

        this.dunkGlyph();

    }


    public void dunkGlyph() throws InterruptedException {
        telemetry.addData("Left target:", mecanumDrive.leftFront.getTargetPosition());
        telemetry.addData("Left current:", mecanumDrive.leftFront.getCurrentPosition());
        telemetry.update();
        if (glyphPos == 'l') {
            if(this.allianceColor == RED) {
                mecanumDrive.moveEncoderStraight(42, 0.2);
            }else{
                mecanumDrive.moveEncoderStraight(-24, 0.2);
            }
        }
        else if (glyphPos == 'c') {
            if(this.allianceColor == RED) {
                mecanumDrive.moveEncoderStraight(35, 0.2);
            }else{
                mecanumDrive.moveEncoderStraight(-31, 0.2);
            }
        }
        else {
            if(this.allianceColor == RED) {
                mecanumDrive.moveEncoderStraight(28, .2);
            }else{
                mecanumDrive.moveEncoderStraight(-40, .2);
            }
        }
        runtime.reset();
        while(!mecanumDrive.encoderDone()&&opModeIsActive()&&!timedout(7000)){}

        jewelArm.store();

        mecanumDrive.brake();
        twait(200);
        if(this.allianceColor == RED){
            mecanumDrive.encoderTurn(92, .6);
        }else{
            mecanumDrive.encoderTurn(97, .2);
        }

        runtime.reset();
        while(!mecanumDrive.encoderDone() && opModeIsActive() && !timedout(3000)){}

        mecanumDrive.brake();
        glyphMechanism.flipUp();

        mecanumDrive.rawMove(0.5,0.5,0.5,0.5);
        twait(800);
        mecanumDrive.brake();

        glyphMechanism.releaseTop();
        glyphMechanism.releaseBottom();

        twait(400);

        mecanumDrive.moveEncoderStraight(-5, 0.4);
        runtime.reset();
        while(!mecanumDrive.encoderDone() && opModeIsActive() && !timedout(2000)){}

        mecanumDrive.rawMove(0.6,0.6,0.6,0.6);
        twait(800);
        mecanumDrive.brake();


        mecanumDrive.moveEncoderStraight(-4, .25);
        while(!mecanumDrive.encoderDone() && opModeIsActive() && !timedout(3000)){}
        mecanumDrive.brake();


    }

    /**
     * Puts down arm, color senses the balls, spins to hit the correct ball off.
     * @throws InterruptedException
     */
    public void hitJewel() throws InterruptedException {
        jewelArm.raise();
        twait(300);
        jewelArm.lower();
        twait(1000);
        jewelArm.hitJewel(allianceColor == RED);

        twait(400);
        jewelArm.raise();
    }


    /**
     * uses Vuforia to detect the pattern, sets a class variable 'glyphPos' to l, r, or c depending on what it sees.
     */
    public void detectPattern() {
        RelicRecoveryVuMark vuMark;
        while (opModeIsActive() && runtime.milliseconds() < 5000) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    glyphPos = 'l';
                    telemetry.addData("put", "left", vuMark);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT){
                    glyphPos = 'r';
                    telemetry.addData("put", "right", vuMark);
                } else{
                    glyphPos = 'c';
                    telemetry.addData("put", "center", vuMark);
                }
                telemetry.update();
                break;
            } else {
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }
        }
    }


    /**
     * Puts glyph into the tower based on what 'glyphPos' is.
     */
    public void twait(long millis) throws InterruptedException{
        Thread.sleep(millis);
    }
    public boolean timedout(long millis){
        return runtime.milliseconds() > millis;
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}

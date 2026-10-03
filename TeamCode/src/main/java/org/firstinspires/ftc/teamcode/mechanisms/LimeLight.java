package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class LimeLight {
    public Limelight3A limelight;
    public IMU imu;

    public void init(HardwareMap hwmap) {
        // Correção: Usa o parâmetro 'hwmap' em vez de 'hardwareMap'
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        imu = hwmap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight.start();
    }

    /**
     * Obtém os resultados mais recentes de visão da Limelight.
     * Use este método na sua OpMode para passar o resultado à Turret.
     */
    public LLResult getLatestResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }

    /**
     * Alterna a pipeline da Limelight.
     */
    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }
}
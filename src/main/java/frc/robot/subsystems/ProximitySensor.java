package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

public class ProximitySensor {
    private I2C proximity = new I2C(I2C.Port.kOnboard, 0x31);

    public ProximitySensor() {
        
    }

    public double[] getDistances() {
        byte[] buffer = new byte[8];
        proximity.read(0, 8, buffer);

        double[] results = new double[4];

        results[0] = (buffer[0] & 0xff) | ((buffer[1] & 0xff) << 8);
        results[1] = (buffer[2] & 0xff) | ((buffer[3] & 0xff) << 8);
        results[2] = (buffer[4] & 0xff) | ((buffer[5] & 0xff) << 8);
        results[3] = (buffer[6] & 0xff) | ((buffer[7] & 0xff) << 8);

        return results;
    }
}
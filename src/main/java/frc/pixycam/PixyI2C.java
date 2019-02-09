package frc.pixycam;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class PixyI2C extends PixyCam {
	Port port = Port.kOnboard;
	I2C I2CBus = new I2C(port, 0x54);
	@Override
	public PixyPacket request(PixyPacket packet, int returnDataLength) {
		byte[] packetBuffer = packet.getPacket();
		byte[] rawData = new byte[returnDataLength];
		System.out.println("Sent Data: " + packetBuffer.length);
		for(int i = 0; i < packetBuffer.length; i++) {
			byte num = packetBuffer[i];
			char[]hexDigits = new char[2];
			hexDigits[0] = Character.forDigit((num >> 4) & 0xF, 16);
			hexDigits[1] = Character.forDigit((num & 0xF), 16);
			System.out.println(new String(hexDigits));
			
		}
		I2CBus.transaction(packetBuffer,packetBuffer.length,rawData,rawData.length);
		System.out.println("Recieved Data: " + rawData.length);
		for(int i = 0; i < rawData.length; i++) {
			byte num = rawData[i];
			char[]hexDigits = new char[2];
			hexDigits[0] = Character.forDigit((num >> 4) & 0xF, 16);
			hexDigits[1] = Character.forDigit((num & 0xF), 16);
			System.out.println(new String(hexDigits));
			
		}
		packet.returnedData = rawData;
		return null;
	}
}

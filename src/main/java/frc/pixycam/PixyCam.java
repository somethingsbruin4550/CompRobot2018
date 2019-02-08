package frc.pixycam;

public abstract class PixyCam {
	public abstract PixyPacket request(PixyPacket packet);
	public void setLamp(boolean upper, boolean lower) {
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x16);
		byte[] data = {(byte) (upper?0x01:0x00), (byte) (lower?0x01:0x00)};
		packet.setPayload(data);
		request(packet);
	}
	public void setLED(byte R, byte G, byte B) {
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x14);
		byte[] data = {R, G, B};
		packet.setPayload(data);
		request(packet);
	}
	public void getRGB(int x, int y, int saturate){
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x70);
		byte[] data = {(byte) x, (byte) y, (byte) saturate};
		packet.setPayload(data);
		request(packet);
	}
}

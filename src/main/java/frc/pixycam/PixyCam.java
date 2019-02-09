package frc.pixycam;

public abstract class PixyCam {
	public abstract PixyPacket request(PixyPacket packet, int returnDataLength);
	public void setLamp(boolean upper, boolean lower) {
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x16);
		byte[] data = {(byte) (upper?0x01:0x00), (byte) (lower?0x01:0x00)};
		packet.setPayload(data);
		request(packet, 0);
	}
	public void setLED(byte R, byte G, byte B) {
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x14);
		byte[] data = {R, G, B};
		packet.setPayload(data);
		request(packet, 0);
	}
	public void getRGB(int x, int y, int saturate){
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x70);
		byte[] data = {(byte) x, (byte) y, (byte) saturate};
		packet.setPayload(data);
		request(packet, 7);
	}
	public int getGrayscale(int x, int y, int saturate){
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x70);
		byte[] data = {(byte) x, (byte) y, (byte) saturate};
		packet.setPayload(data);
		request(packet, 7);
		return ((int) (packet.returnedData[6] + packet.returnedData[7] + packet.returnedData[8]) / 3);
	}
	public int[] getResolution(){
		PixyPacket packet = new PixyPacket();
		packet.setType((byte) 0x0D);
		byte[] data = {0x00};
		packet.setPayload(data);
		request(packet,2);
		//Parsing Resolution from returnedData:
		//int[0] = x, int [1] = y
		int[] resolution = {(int) (packet.returnedData[6] + packet.returnedData[7]), (int) (packet.returnedData[8] + packet.returnedData[9])};
		return resolution;
	}
	public int[][] getImage(){
		int[] resolution = getResolution();
		int[][] image = new int[resolution[0]][resolution[1]]; //int[0] = x, int [1] = y
		for(int x = 0; x < resolution[0]; x++){
			for(int y = 0; y < resolution[1]; y++){
				image[x][y] = getGrayscale(x, y, 0);
			}
		}
		return image;
	}
}

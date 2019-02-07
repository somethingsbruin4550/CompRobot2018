package pixycam;

public class PixyPacket {
	byte[] packetBuffer = new byte[8];
	byte[] syncBuffer = {(byte) 0xAE,(byte) 0xC1};
	byte type = 0;
	byte[] payload = new byte[0];
	public PixyPacket() {
		
	}
	public byte[] getPacket() {
		byte[] packet = new byte[syncBuffer.length + 2 + payload.length];
		for(int i = 0;i < syncBuffer.length;i++) {
			packet[i] = syncBuffer[i];
		}
		packet[syncBuffer.length] = type;
		packet[syncBuffer.length + 1] = (byte) (((byte) payload.length) & (byte) 0xFF);
		for(int i = 0;i < payload.length;i++) {
			packet[syncBuffer.length + 2 + i] = payload[i];
		}
		
		return packet;
	};
	public void importRecvPacket(byte[] packet) {
		
	}
	public byte getType() {
		return type;
	}
	public void setType(byte type) {
		this.type = type;
	}
	public byte[] getPayload(){
		return payload;
	}
	public int getPayloadLength() {
		return payload.length;
	}
	public void setPayload(byte[] payload) {
		this.payload = payload;
	}
}

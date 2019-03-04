package frc.pixycam;

public class PixyUtil {
    public static void printHexList(byte[] data){
        for (int i = 0; i < data.length; i++) {
            byte num = data[i];
            char[] hexDigits = new char[2];
            hexDigits[0] = Character.forDigit((num >> 4) & 0xF, 16);
            hexDigits[1] = Character.forDigit((num & 0xF), 16);
            System.out.println("0x" + new String(hexDigits));
        }
    }
    public static String printHex(byte data){
        char[] hexDigits = new char[2];
        hexDigits[0] = Character.forDigit((data >> 4) & 0xF, 16);
        hexDigits[1] = Character.forDigit((data & 0xF), 16);
        //System.out.println("0x" + new String(hexDigits));
        return "0x" + new String(hexDigits);
    }
}
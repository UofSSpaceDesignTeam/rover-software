
public class Arduino {
	TwoWaySerialComm port = new TwoWaySerialComm("/dev/ttyUSB0");
	
	public String sendMessage(String in){
		port.writeMessage(in);
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return port.readMessage();
	}
	
	public static void main(String[] args){
		Arduino a = new Arduino();
		System.out.println(a.sendMessage("Hello\n"));
	}
}

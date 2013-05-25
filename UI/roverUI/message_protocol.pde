
// protocol for sending messages to rover
// literally copied from the rover to arduino message protocol then modified message names and values

public abstract class MessageProtocol {
  
  public static final byte ID1_POWER ='P';
  public static final byte ID1_DEBUG ='D';
  public static final byte ID1_MOTORS ='M';

  public static final byte ID1_DIGGER ='G';
  
  public static final byte ID2_CHECK ='C';
  public static final byte ID2_DEBUG ='B';
  public static final byte ID2_OFF ='O';
  public static final byte ID2_EMERGENCY_OFF ='E';
  public static final byte ID2_RAISE ='R';
  public static final byte ID2_ENABLE_DISABLE ='E';
  public static final byte ID2_ROTATION ='S';

  public static final byte START_BYTE ='#';
  public static final byte END_BYTE ='\n';
  public static final byte FIRST_DATA_BYTE = 5;
}

public class Message {
  byte[] message;
  
  public void Message(byte[] msg)
  {
    this.message = msg;
  }
  
  public void Message(byte id1, byte id2, byte[] data)
  {
    message = new byte[data.length +2+2+1+1];
    message[0] = MessageProtocol.START_BYTE;
    message[1] = id1;
    message[2] = id2;
    message[3] = (byte)(data.length / 256);
    message [4] =  (byte)(data.length % 256);
    for(int i = 0; i < data.length; i++){
      message[i+MessageProtocol.FIRST_DATA_BYTE] = data[i];
    }
    message[message.length - 1] = MessageProtocol.END_BYTE; 
  }
  public void sendMessage()
  {
    client.write(message);
  }
  public void recieveMessage()
  {
    message = client.readBytes();
  }
  public byte[] getMessge()
  {
    return this.message;
  }
  public byte getId1()
  {
    return this.message[1];
  }
  public byte getId2()
  {
    return this.message[2];
  }
  public byte[] getData()
  {
    int dataLength = message[3] * 256 + message[4];
    
    byte[] data = new byte[dataLength];
    for (int i = 0; i < dataLength; i++){
      data[i] = message[i + MessageProtocol.FIRST_DATA_BYTE];
    }
    return data;
  }
}

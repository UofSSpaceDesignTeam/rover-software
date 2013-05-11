Message outMessage = new Message();
Message inMessage = new Message();
byte[] speed = new byte[2];
boolean move_enable_confirmed = false;
boolean move_disable_confirmed = true;

void sendMessages(){
  if (move_upButton.isActive() || move_downButton.isActive() || move_leftButton.isActive() || move_rightButton.isActive()){
    println("GO!");
    //println(int(byte(200)));
    move_upButton.deactivate();
   
    //speed[0] = byte(motor1Speed);
    //speed[1] = byte(motor2Speed);
    
    speed[0] = byte(motor1Speed);
    speed[1] = byte(motor2Speed);
    println(speed[0]);
    println(speed[1]);

    outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
    outMessage.sendMessage(); 
 
 
   
  }
  if (stop_moveButton.isActive()){
    println("STOP");
    //println(int(byte(200)));
    stop_moveButton.deactivate();//temporary loacation, should be deactivated upon confirmation from rover
    
    speed[0] = byte(127);
    speed[1] = byte(127);
    println(motor1Speed);
    println(motor2Speed);

    
    outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
    outMessage.sendMessage();    
    //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,null);  
  }
  if (enable_moveButton.isActive()){
    if(move_enable_confirmed == false){
    println("Enable move");
    //println(int(byte(200)));
    byte[] data = new byte[1];
    data[0] = '1';
   
    //speed[0] = byte(127);
    //speed[1] = byte(127);
    //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
    //outMessage.sendMessage();    
    outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,data);
    outMessage.sendMessage(); 
    move_disable_confirmed = false;
    move_enable_confirmed = true;
    }    
  } else{
    if(move_disable_confirmed == false){
    println("Disable move");
    //println(int(byte(200)));
    byte[] data = new byte[1];
    data[0] = byte(0);
    
    //outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ROTATION,speed);
    //outMessage.sendMessage();    
    outMessage.Message(MessageProtocol.ID1_MOTORS,MessageProtocol.ID2_ENABLE_DISABLE,data);
    outMessage.sendMessage(); 
    move_disable_confirmed = true;
    move_enable_confirmed = false;
    }


  }    
}

void getMessages(){
  if(client.available()>0){
    inMessage.recieveMessage();
  } 
}


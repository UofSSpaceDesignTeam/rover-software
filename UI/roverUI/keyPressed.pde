void keyPressed() {
  
  if (key == CODED) {
    if (keyCode == UP) {
       move_upButton.activate();
    } else if (keyCode == DOWN) { 
       move_downButton.activate();
    } else if (keyCode == LEFT) { 
       move_leftButton.activate();
    }else if (keyCode == RIGHT) { 
       move_rightButton.activate();
    }
    
  } else {
    if (key == 'A') {
   
    } else { 

    }
  }
  
}




void keyReleased(){
  println("key: "+key);
  println("keyCode: "+keyCode);
  
  if (key == CODED) {
    if (keyCode == UP) {
       move_upButton.deactivate();
    } else if (keyCode == DOWN) { 
       move_downButton.deactivate();
    } else if (keyCode == LEFT) { 
       move_leftButton.deactivate();
    }else if (keyCode == RIGHT) { 
       move_rightButton.deactivate();
    }else if (keyCode == 33) { 
      //speedbar.locked = true;
      speedbar.newspos = speedbar.newspos + 5/speedbar.ratio;
      //speedbar.spos = speedbar.spos + 5/speedbar.ratio;
      //speedbar.locked = false;
      
    }
    
  } else {
    if (key == 'A') {
   
    } else { 

    }
  }  
  
  
  
}

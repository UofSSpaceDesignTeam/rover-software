void draw() {
 background(00,195,50);
 noteY=15;
 Client getclient = myServer.available();

  if(inmessage != null){
    fill(0);
     text("Client says: "+ char(inmessage[0]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ char(inmessage[1]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ char(inmessage[2]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ int(inmessage[3]),noteX,noteY,150,15);
    noteY= noteY+15; 
    text("Client says: "+ int(inmessage[4]),noteX,noteY,150,15);
     noteY= noteY+15; 

    for(int i = 5; i < inmessage.length-1; i++){
     text("Client says: "+ int(inmessage[i]),noteX,noteY,150,15);
     noteY= noteY+15; 
    }
     text("Client says: "+ char(inmessage[inmessage.length-1]),noteX,noteY,150,15);

  }
  if(getclient != null){

  if(getclient.available()>0){
    if(inmessage!=null){
     lastin = inmessage;
    }
   inmessage = getclient.readBytes();
  }
 }
}

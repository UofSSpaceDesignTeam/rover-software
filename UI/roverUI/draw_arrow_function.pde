void arrow(int x1, int y1, int x2, int y2) {
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
} 

//code taken from reply #2 of http://processing.org/discourse/beta/num_1219607845.html 


const int leftWheel = 11;
const int rightWheel = 10;

void setup() {
  Serial.begin(9600);

}

void loop() {
  if (Serial.available() > 0) {
    readSerial = Serial.read();
    move(readSerial);
  }

}

void move(dir){
  if (dir == "left"){
 
  } elseif (dir == "right) {
    
  } else {
    
  }
}

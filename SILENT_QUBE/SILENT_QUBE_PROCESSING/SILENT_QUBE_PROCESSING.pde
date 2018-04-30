import processing.serial.*;
import http.requests.*;

int ENDLINE = 10;    // Linefeed in ASCII
Serial myPort;  // The serial port
int yaw, pitch, roll = 0;
boolean sender = false;
boolean rollSender = false;
boolean yawSender = false;
boolean yawSender2 = false;
String myString ="";
int currState, prevState, currStateRoll, prevStateRoll, currStateYaw, prevStateYaw;
int value = 0;
String[] places = {"LINK_TO_IFTTT_API_CALL_1", 
  "LINK_TO_IFTTT_API_CALL_2"};         

static String YAW = "y";
static String PITCH = "p";
static String ROLL = "r";

int lastxPos=1;
int lastheight=0;
int xPos = 1;

void setup() {

  size(450, 400);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.clear();
  // Throw out the first reading, in case we started reading 
  // in the middle of a string from the sender.
  myString = myPort.readStringUntil(ENDLINE);
  myString = null;
  // SEND FIRST REQUEST CODE TO START COMMUNICATION
  myPort.write("Z");
  background(0);
}


void draw() {  

  // WAIT FOR RESPONSE WITH PROPER RESPONSE CODE
  if (myPort.available() > 0 && myPort.readChar() == 'Q') {   
    // READ YAW PITCH AND ROLL VALUES
    yaw = getValue(YAW);
    pitch = getValue(PITCH);
    roll = getValue(ROLL);

    // IF FINISHED READING 
    if (myPort.readChar() == 'X') {
      println("Y:"+yaw+" P:"+pitch+" R:"+roll);

      currState = pitch;
      currStateRoll = roll;
      currStateYaw = yaw;

      //CHECK AND UPDATE PITCH STATUS
      sender = currState > 50 && prevState < 50; 

      //CHECK AND UPDATE ROLL STATUS
      rollSender = currStateRoll > 50 && prevStateRoll < 50;

      yawSender = currStateYaw > 50 && prevStateYaw < 50;
      yawSender2 = currStateYaw <0  && prevStateYaw > 0;

      prevState = pitch;
      prevStateRoll = roll;
      prevStateYaw = yaw;

      //// IF PITCH SWITCHED
      //if (sender) {
      //  print("Twisted Pitch");
      //  GetRequest get = new GetRequest(places[0]);
      //  get.send();
      //  // RESET
      //  sender = false;
      //  rollSender = false;
      //}

      // IF ROLL SWITCHED
      //if (rollSender) {
      //  print("Twisted Roll");
      //  //GetRequest get = new GetRequest(places[1]);
      //  //get.send();
      //  // RESET
      //  sender = false;
      //  rollSender = false;
      //}

      //// IF YAW SWITCHED
      if (yawSender) {
        print("Yaw Ringer");
        GetRequest get = new GetRequest(places[0]);
        get.send();
        // RESET
        sender = false;
        yawSender = false;
         myPort.write("R");
      }

      ////// IF YAW SWITCHED
      if (yawSender2) {
        print("Yaw Silent");
        GetRequest get = new GetRequest(places[1]);
        get.send();
        // RESET
        sender = false;
        yawSender2 = false;
        myPort.write("S");
      }

      drawStuff(yaw, color(255, 0, 0));
      drawStuff(pitch, color(0, 255, 0));
      drawStuff(roll, color(12, 120, 255));
    }

    // SEND NEXT REQUEST CHARACTER
    myPort.write("Z");
    //println("Sent new requesst");
  }
}


void drawStuff(int val, color myColor) {
  int h = int(map(val, -100, 100, 0, height));
  stroke(myColor);
  strokeWeight(4);   
  point(lastxPos, lastheight);
  lastxPos= xPos;
  lastheight= int(height-h);
  // IF AT EDGE
  if (xPos >= width) {
    xPos = 0;
    lastxPos= 0;
    background(0);
  } else {
    // MOVE FWD
    xPos++;
  }
}


int getValue(String axis) {
  if (myPort.readChar() == axis.charAt(0)) {
    myString = myPort.readStringUntil(axis.charAt(0));
    if (myString != null) {
      value = Integer.parseInt(myString.replace(axis, "").trim());
    }
  }
  return value;
}


int ASCIItoInt(char c)
{
  if ((c >= '0') && (c <= '9'))
    return c - 0x30; // Minus 0x30
  else if ((c >= 'A') && (c <= 'F'))
    return c - 0x37; // Minus 0x41 plus 0x0A
  else if ((c >= 'a') && (c <= 'f'))
    return c - 0x57; // Minus 0x61 plus 0x0A
  else
    return -1;
}
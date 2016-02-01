//////////////////////////////////////////////////////////////////////////////////////////////
//*******Dan Miller's super-excellent computer to microcontroller data transfer code!*******//
//////////////////////////////////////////////////////////////////////////////////////////////
/*
Usage guide:
 Output:
 Put variable names that are to be sent to the computer in the outNames struct, and update the
 OUT_NUM def to accurately represent how many variables are placed into the struct
 Input:
 Put variable names that are to be sent to the computer in the inNames struct, and update the
 IN_NUM def to accurately represent how many variables are placed into the struct
 Value Access:
 Use the pre-defined instances of either inNames or outNames(inGroup and outGroup, respecively)
 to access the variables named previously. The inGroup values can be modified, but will be
 overwritten on the next update received from the computer. The outGroup values should be
 modified: again, these are the values which will be sent to the computer side.
 Other Constants:
 UPDATE_RATE - the minimum time in ms between updates sent to the server
 DEBUG - true for prints to serial, false otherwise
 LOCAL_PORT - the port to be used by the device
 mac[] - the MAC address for this device. Should be set manually
 ip - the IP to be used for this device
 Necessary Method Calls:
 setupSync() - must be called sometime during the setup() function. if DEBUG, initializes Serial
 updateSync() - should be called very often in loop(). Checks for updates in the network buffer,
 and if UPDATE_RATE ms have passed since last update will send an update back to the computer
 */

#include "rovCOM.h"

#define UDP_TX_PACKET_MAX_SIZE 256

inNames_t inGroup;
outNames_t outGroup;

int UPDATE_RATE;
int LOCAL_PORT;

// buffers for receiving and sending data
byte packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

//function prototypes
void updateSegment(byte index, byte * data);
void proccessPacket(byte buffer[], int size);
void handleDataOut();
void updateOutStream();


void setupSync(byte mac[], IPAddress ip, int updateRate_ms, int localPort){
  //set global variables
  UPDATE_RATE = updateRate_ms;
  LOCAL_PORT = localPort;
  
  // start the Ethernet and UDP:
  Ethernet.begin(mac,ip);
  Udp.begin(LOCAL_PORT);

# if(DEBUG)
    Serial.begin(115200);
    Serial.println(Ethernet.localIP());
# endif

}

//Function to check if new UDP data is available and to send data cyclically
//Returns 1 if data was received
//Returns 0 if there was no new data
// TODO: Move the data output to a timer interrupt
int updateSync(){
  int dataReceived = 0;
  int packetSize = Udp.parsePacket();
  static int time = 0; //ms between updates
  
  if(packetSize){
    // read the packet into packetBufffer
    proccessPacket(packetBuffer, Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE));
    dataReceived = 1;
  }

  //TODO: Move this to use a timer interrupt instead of polling millis()
  if(time + UPDATE_RATE < millis()){
    time = millis();
    handleDataOut();
  }

  return dataReceived;
}

/*
 * Returns pointer to array of all output data formatted to be sent directly over UDP
 */
const int outSegLen = (2 + sizeof(int32_t));
const int outStreamLen = (OUT_NUM) * outSegLen;
byte outStream[outStreamLen];
void updateOutStream(){
  int * outs = (int*)(&outGroup);
  for(byte i = 0; i < OUT_NUM; i++){
    outStream[i * outSegLen] = 0;
    outStream[(i * outSegLen) + 1] = (byte)(i + 1);
    memcpy(outStream + (i*outSegLen) + 2, outs + i, sizeof(int32_t));
  }
}

/*
 * Checks the network input buffer, processes all updates
 * Sends updates for all data needed
 *     For now, will send all data values regardless of whether they changed
 */
void handleDataOut(){
  // send a reply, to the IP address and port that sent us the packet we received
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  updateOutStream();

  Udp.write(outStream, outStreamLen);
  Udp.endPacket();
}

/*
 * Disasemble the buffer stream into individual updates
 * Stream format:
 * 0x00 <Data index + 1> <Data>x4  Repeat
 * buffer: contains raw byte stream from UDP
 */
void proccessPacket(byte buffer[], int size){
  int i = 0;
  while(i < size){
    if(buffer[i] == 0 && (size - i) >= 6 && buffer[i+1] <= IN_NUM){
      updateSegment(buffer[i + 1] - 1, buffer + i + 2);
      i += 6;
    }
    else{
      i++;
    }
  }
#if(DEBUG)
    int * ins = (int*)(&inGroup);
    for(byte i = 0; i < IN_NUM; i++){
      Serial.print(*(ins + i));
      Serial.print(", ");
    }
    Serial.println();
#endif
}

/*
 * Update the peice of data just received. Input format:
 * index: Bit 0-7 : index in the data array
 * data:  pointer to byte array containing the dissasembled
 *         float or int
 */
void updateSegment(byte index, byte * data){
  if(index >= IN_NUM){
    return;
  }
  memcpy((int*)(&inGroup) + index, data, sizeof(int32_t));
}

#include <SoftwareSerial.h>

#define GPS_SV_MAXSATS 28

// Connect the GPS RX/TX to arduino pins 3 and 5
SoftwareSerial serial = SoftwareSerial(10,9);

const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 };
const unsigned char NAV_STATUS_HEADER[] = { 0x01, 0x03 };
const unsigned char NAV_TIMEGPS_HEADER[] = { 0x01, 0x20 };
const unsigned char NAV_POSECEF_HEADER[] = { 0x01, 0x01 };
const unsigned char NAV_SVINFO_HEADER[] = { 0x01, 0x30 };

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
//  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x20,0x00,0x01,0x00,0x00,0x00,0x00,0x31,0x90, //NAV-TIMEGPS on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x12,0xB7, //NAV_POSECEF on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x30,0x00,0x01,0x00,0x00,0x00,0x00,0x41,0x00 , //NAV-SVINFO on

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

enum _ubxMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_TIMEGPS,
  MT_NAV_POSECEF,
  MT_NAV_SVINFO 
};

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

struct NAV_STATUS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned char gpsFix;
  char flags;
  char fixStat;
  char flags2;
  unsigned long ttff;
  unsigned long msss;
};

struct NAV_TIMEGPS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW; //GPS Millisecond time of Week
  signed long fTOW; //Fractional Nanoseconds remainder of rounded ms above
  signed short week;  //GPS week(GPS time)
  signed char leapS;  //leap seconds(GPS-UTC)
  char valid; //valid flags
  unsigned long tAcc; //time accuracy estimate
};

struct NAV_POSECEF {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW; //GPS Millisecond time of Week
  signed long ecefX; 
  signed long ecefY;
  signed long ecefZ;
  unsigned long pAcc; //position accuracy estimate
};

struct NAV_SVINFO {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW; //GPS Millisecond time of Week
  unsigned char numCh;  //Number of channels 
  char globalFlags; //Bitmask - chipGen (chip hardware generation)
  unsigned short served2; //reserved
  //repetead block
  unsigned char chn[GPS_SV_MAXSATS]; //channel number
  unsigned char svid[GPS_SV_MAXSATS];  //satellite ID
  char flags[GPS_SV_MAXSATS];
  char quality[GPS_SV_MAXSATS];
  unsigned char cno[GPS_SV_MAXSATS]; //carrier to noise ratio
  signed char elevation[GPS_SV_MAXSATS]; 
  signed short azimuth[GPS_SV_MAXSATS];
  signed long pseudoRangeResidual[GPS_SV_MAXSATS]; //[cm]
};

union UBXMessage {
  NAV_POSLLH navPosllh;
  NAV_STATUS navStatus;
  NAV_TIMEGPS navTimegps;
  NAV_POSECEF navPosecef;
  NAV_SVINFO navSvinfo;
};

UBXMessage ubxMessage;

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
    //ensure unsigned byte range; mask the checksum to be one byte
  CK[0] = CK[0] & 0xFF;
  CK[1] = CK[1] & 0xFF; 
}


// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}


// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);
  unsigned short payloadLen;

  unsigned char numChSvinfo;

  //memset();

  while ( serial.available() ) {
    
    byte c = serial.read();    
    //Serial.write(c);
    
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;

      fpos++;
      
      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_POSLLH_HEADER) ) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH);
        }
        else if ( compareMsgHeader(NAV_STATUS_HEADER) ) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(NAV_STATUS);
        }
        else if ( compareMsgHeader(NAV_TIMEGPS_HEADER) ) {
          currentMsgType = MT_NAV_TIMEGPS;
          payloadSize = sizeof(NAV_TIMEGPS);
        }
         else if ( compareMsgHeader(NAV_POSECEF_HEADER) ) {
          currentMsgType = MT_NAV_POSECEF;
          payloadSize = sizeof(NAV_POSECEF);
        }
        else if ( compareMsgHeader(NAV_SVINFO_HEADER) ) {
          currentMsgType = MT_NAV_SVINFO;
          //populateSvinforepeatedBlock();
          payloadSize = sizeof(NAV_SVINFO);
//          Serial.println("current msg type:");
//          Serial.println(currentMsgType);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

//      if (fpos == 5) {
//        payloadLen = ((unsigned short*)(&ubxMessage))[fpos-2];
//        Serial.println("fpos 5");
//        Serial.print("msg type1:");
//        Serial.print(currentMsgType);
//        Serial.println();
//        Serial.print("payload Len1:");
//        Serial.print(payloadLen);
//        Serial.println();       
//      }
//
//      if (fpos == 6) {
//          payloadLen += (unsigned short)(((unsigned char*)(&ubxMessage))[fpos-2] << 8);
//          Serial.println("fpos 6");
//          Serial.print("msg type2:");
//          Serial.print(currentMsgType);
//          Serial.println();
//          Serial.print("payload Len2:");
//          Serial.print(payloadLen);
//          Serial.println(); 
//      }

//      if ((fpos == 7) && (currentMsgType == MT_NAV_POSECEF)) {
//        Serial.println();
//        Serial.print("POSECEF LEN: ");
//        Serial.print(ubxMessage.navPosecef.len);
//        Serial.println();
//        Serial.print("POSECEF payloadsize: ");
//        Serial.print(payloadSize);
//        Serial.println();
//        Serial.print("----------------------");
//      }
//      else if ((fpos == 7) && (currentMsgType == MT_NAV_TIMEGPS)) {
//        Serial.println();
//        Serial.print("TIMEGPS LEN: ");
//        Serial.print(ubxMessage.navTimegps.len);
//        Serial.println();
//        Serial.print("TIMEGPS payloadsize: ");
//        Serial.print(payloadSize);
//        Serial.println();
//        Serial.print("----------------------");
//      }
//      else if ((fpos == 7) && (currentMsgType == MT_NAV_STATUS)) {
//        Serial.println();
//        Serial.print("STATUS LEN: ");
//        Serial.print(ubxMessage.navStatus.len);
//        Serial.println();
//        Serial.print("STATUS payloadsize: ");
//        Serial.print(payloadSize);
//        Serial.println();
//        Serial.print("----------------------");
//      }
//      else if ((fpos == 7) && (currentMsgType == MT_NAV_POSLLH)) {
//        Serial.println();
//        Serial.print("POSLLH LEN: ");
//        Serial.print(ubxMessage.navPosllh.len);
//        Serial.println();
//        Serial.print("POSLLH payloadsize: ");
//        Serial.print(payloadSize);
//        Serial.println();
//        Serial.print("----------------------");
//      }
            
      if ((fpos == 7) && (currentMsgType == MT_NAV_SVINFO)) {
        numChSvinfo = ((unsigned char*)(&ubxMessage))[fpos-2];
//        ubxMessage.navSvinfo.svid = (unsigned char*)malloc(sizeof(unsigned char)*numChSvinfo); 
//        ubxMessage.navSvinfo.flags = (char*)malloc(sizeof(char)*numChSvinfo);
//        ubxMessage.navSvinfo.quality = (char*)malloc(sizeof(char)*numChSvinfo);
//        ubxMessage.navSvinfo.cno = (unsigned char*)malloc(sizeof(unsigned char)*numChSvinfo); 
//        ubxMessage.navSvinfo.elevation = (signed char*)malloc(sizeof(signed char)*numChSvinfo); 
//        ubxMessage.navSvinfo.azimuth = (signed short*)malloc(sizeof(signed short)*numChSvinfo);
//        ubxMessage.navSvinfo.pseudoRangeResidual = (signed long*)malloc(sizeof(signed long)*numChSvinfo);
        //payloadSize = sizeof(NAV_SVINFO);
        //payloadSize = 8+12*numChSvinfo;

//        Serial.println();
//        Serial.print("numChSvinfo: ");
//        Serial.print(numChSvinfo);
//        Serial.println();
//        Serial.print("SVINFO len : ");
//        Serial.print(ubxMessage.navSvinfo.len);
//        Serial.println();
//        Serial.print("SVINFO payloadsize: ");
//        Serial.print(payloadSize);
//        Serial.println();
//        Serial.print("----------------------");
      }
        
      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
        
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          //Serial.print(currentMsgType);
          fpos = 0; 
          
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          //Serial.println(currentMsgType);
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  return MT_NONE;
}

void setup()
{
  serial.begin(9600);
  Serial.begin(9600);

  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    serial.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

long lat;
long lon;

void loop() {
  int msgType = processGPS();
  if ( msgType == MT_NAV_POSLLH ) {
    Serial.println();
    Serial.println();
    Serial.print("--- NAV_POSLLH ---");
    Serial.println();
    Serial.print("iTOW:");      Serial.print(ubxMessage.navPosllh.iTOW);
    Serial.println();
    Serial.print("lat/lon: "); Serial.print(ubxMessage.navPosllh.lat/10000000.0f); Serial.print(","); Serial.print(ubxMessage.navPosllh.lon/10000000.0f);
    Serial.println();
    Serial.print("hAcc: ");    Serial.print(ubxMessage.navPosllh.hAcc/1000.0f);
    Serial.println();
    Serial.println("-------------------");
    Serial.println();
  }
  else if ( msgType == MT_NAV_STATUS ) {
    Serial.println();
    Serial.println();
    Serial.print("--- NAV_STATUS ---");
    Serial.println();
    Serial.print("gpsFix:");    Serial.print(ubxMessage.navStatus.gpsFix);
    Serial.println();
    Serial.print("iTow:");    Serial.print(ubxMessage.navStatus.iTOW);
    Serial.println();
    Serial.print("flags:");    Serial.print(ubxMessage.navStatus.flags);
    Serial.println();
    Serial.print("fixStat:");    Serial.print(ubxMessage.navStatus.fixStat);
    Serial.println();
    Serial.print("flags2:");    Serial.print(ubxMessage.navStatus.flags2);
    Serial.println();
    Serial.print("ttff:");    Serial.print(ubxMessage.navStatus.ttff);
    Serial.println();
    Serial.println("-------------------");
    Serial.println();
  }
  else if ( msgType == MT_NAV_TIMEGPS ) {
    Serial.println();
    Serial.println();
    Serial.print("--- NAV_TIMEGPS ---");
    Serial.println();
    Serial.print("iTOW:");      Serial.print(ubxMessage.navTimegps.iTOW);
    Serial.println();
    Serial.print("tAcc:");      Serial.print(ubxMessage.navTimegps.tAcc);
    Serial.println();
    Serial.print("week:");      Serial.print(ubxMessage.navTimegps.week);
    Serial.println();
    Serial.println("-------------------");
    Serial.println();
  }
  else if ( msgType == MT_NAV_POSECEF ) {
    Serial.println();
    Serial.println();
    Serial.print("--- NAV_POSECEF ---");
    Serial.println();
    Serial.print("iTOW:");      Serial.print(ubxMessage.navPosecef.iTOW);
    Serial.println();
    Serial.print("ecefX:");      Serial.print(ubxMessage.navPosecef.ecefX);
    Serial.println();
    Serial.print(" cefY:");      Serial.print(ubxMessage.navPosecef.ecefY);
    Serial.println();
    Serial.print("ecefZ:");      Serial.print(ubxMessage.navPosecef.ecefZ);
    Serial.println();
    Serial.println("-------------------");
    Serial.println();
  }
  else if ( msgType == MT_NAV_SVINFO ) {
    Serial.println();
    Serial.println();
    Serial.print("--- NAV_SVINFO ---");
    Serial.println();
    Serial.print("iTOW:");      Serial.print(ubxMessage.navSvinfo.iTOW);
    Serial.println();
    Serial.print("numCh:");      Serial.print(ubxMessage.navSvinfo.numCh);
    Serial.println();
    Serial.print("globalFlags:");      Serial.print(ubxMessage.navSvinfo.globalFlags);
    Serial.println();
    Serial.print("served2:");      Serial.print(ubxMessage.navSvinfo.served2);
    Serial.println();
    for (int i = 0; i < GPS_SV_MAXSATS; i++) {
        Serial.print("****** REPEATED BLOCK INDEX: ****** ");      Serial.print(i);
        Serial.println();
        Serial.print("  chn:");      Serial.print(ubxMessage.navSvinfo.chn[i]);
        Serial.println();
        Serial.print("  svid:");      Serial.print(ubxMessage.navSvinfo.svid[i]);
        Serial.println();
        Serial.print("  flags:");      Serial.print(ubxMessage.navSvinfo.flags[i]);
        Serial.println();
        Serial.print("  quality:");      Serial.print(ubxMessage.navSvinfo.quality[i]);
        Serial.println();
        Serial.print("  cno:");      Serial.print(ubxMessage.navSvinfo.cno[i]);
        Serial.println();
        Serial.print("  elevation:");      Serial.print(ubxMessage.navSvinfo.elevation[i]);
        Serial.println();
        Serial.print("  azimuth:");      Serial.print(ubxMessage.navSvinfo.azimuth[i]);
        Serial.println();
        Serial.print("  pseudoRangeResidual:");      Serial.print(ubxMessage.navSvinfo.pseudoRangeResidual[i]);
        Serial.println();
    }
    Serial.println("-------------------");
    Serial.println();
    Serial.println();
  }
}


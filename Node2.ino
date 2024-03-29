#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;

SoftwareSerial ss(7,6); // Arduino RX, TX to conenct to GPS module.


static void smartdelay(unsigned long ms);

unsigned int count = 1;        //For times count


String datastring1="";        
String datastring2="";        
String datastring3="";
uint8_t datasend[20];     //Used to store GPS data for uploading

char gps_lon[20]={"\0"};  //Storage GPS info
char gps_lat[20]={"\0"}; //Storage latitude
char gps_alt[20]={"\0"}; //Storage altitude
float flat, flon,falt;

static uint8_t mydata[] = "Hello, world! node2";      //For test using.

/* LoRaWAN NwkSKey, network session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially.
   ttn*/
static const PROGMEM u1_t NWKSKEY[16] =  { 0x09, 0x15, 0xfa, 0x7e, 0xec, 0xf2, 0x92, 0x65, 0x4b, 0x89, 0xff, 0x82, 0xd6, 0x7b, 0xe7, 0x83 };
//09 15 fa 7e ec f2 92 65 4b 89 ff 82 d6 7b e7 83
/* LoRaWAN AppSKey, application session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially.
   ttn*/
static const u1_t PROGMEM APPSKEY[16] = { 0x66, 0xeb, 0x87, 0x66, 0x42, 0x3d, 0x72, 0x92, 0x87, 0x49, 0x54, 0x52, 0x0c, 0xbc, 0xde, 0xb8 };
//66 eb 87 66 42 3d 72 92 87 49 54 52 0c bc de b8
/*
 LoRaWAN end-device address (DevAddr)
 See http://thethingsnetwork.org/wiki/AddressSpace
 ttn*/


static const u4_t DEVADDR = 0x0088d23e;
//00 88 d2 3e

/* These callbacks are only used in over-the-air activation, so they are
  left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain).*/
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t initjob,sendjob,blinkjob;

/* Schedule TX every this many seconds (might become longer due to duty
 cycle limitations).*/
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {8, 9, LMIC_UNUSED_PIN},
};

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else {
        GPSRead();
        GPSWrite();
  
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1,datasend,sizeof(datasend)-1,0);
        //LMIC_setTxData2(1, gps_lat, sizeof(gps_lat)-1, 0);
        Serial.println("Packet queued");
        Serial.print("LMIC.freq:");
        Serial.println(LMIC.freq);
        Serial.println("");
        Serial.println("");
        Serial.println("Receive data:");
        
    } 
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(ev);
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println("EV_JOINING");
            break;
        case EV_JOINED:
            Serial.println("EV_JOINED");
            break;
        case EV_RFU1:
            Serial.println("EV_RFU1");
            break;
        case EV_JOIN_FAILED:
            Serial.println("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            Serial.println("EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print("Data Received: ");
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println("EV_LINK_ALIVE");
            break;
         default:
            Serial.println("Unknown event");
            break;
    }
}

void setup() {
     // initialize digital pin  as an output.
    Serial.begin(9600);
    ss.begin(9600);  
    while(!Serial);
    Serial.println("LoRa GPS Example---- ");
    Serial.println("Connect to TTN");
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);    

    /*LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
     Set static session parameters. Instead of dynamically establishing a session
     by joining the network, precomputed session parameters are be provided.*/
    #ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again.*/
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

  //Frekuensi Indo
   // #if defined(CFG_us915)
   //LMIC_setupChannel(0, 920360000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  
    // #endif
  // Disable link check validation
    LMIC_setLinkCheckMode(0);

    for (int i=0; i<9; i++) { // For EU; for US use i<71
      if(i != 0) {
        LMIC_disableChannel(i);
      }
    }
    
    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    do_send(&sendjob);
}

void GPSRead()
{
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  falt=gps.f_altitude();  //get altitude    
  flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;//save six decimal places 
  flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
 // falt == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : falt, 2;
  //Serial.println(flat);
}

void GPSWrite()
{
  /*Convert GPS data to format*/
  datastring1 +=dtostrf(flat, 0, 6, gps_lat); 
  datastring2 +=dtostrf(flon, 0, 6, gps_lon);
  //datastring3 +=dtostrf(falt, 0, 2, gps_alt);
  
  int32_t lng = flon * 10000;
  int32_t lat = flat * 10000;
  
  if(flon!=1000.000000)
  {  
  strcat(gps_lat,",");
  strcat(gps_lat,gps_lon); 
  //strcat(gps_lon,","); 
  //strcat(gps_lon,gps_alt);
  strcpy((char*)datasend,gps_lat); //the format of datasend is longtitude,latitude,altitude
  Serial.print("###########    ");
  Serial.print("NO.");
  Serial.print(count);
  Serial.println("    ###########");
  Serial.println("The longtitude and latitude are:");
  Serial.print("[");
  Serial.print((char*)datasend);
  Serial.print("]");
  Serial.print("");

  count++;
  }

  


  smartdelay(1000);
}


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}


void loop() {
    os_runloop_once();   
}

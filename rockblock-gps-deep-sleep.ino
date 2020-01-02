#include <IridiumSBD.h> // Using own fork of IridiumSBD @ https://github.com/appelflap/IridiumSBD
#include <SoftwareSerial.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <Time.h> // https://www.pjrc.com/teensy/td_libs_Time.html
#include <EEPROM.h> // For writing settings
#include <SPI.h>
#include <SdFat.h>

#define BEACON_INTERVAL 3600 // Time between transmissions
#define ROCKBLOCK_RX_PIN 15
#define ROCKBLOCK_TX_PIN 14
#define ROCKBLOCK_SLEEP_PIN 4
#define ROCKBLOCK_BAUD 19200
#define ROCKBLOCK_SENDRECEIVE_TIME 120
#define GPS_RX_PIN 17
#define GPS_TX_PIN 18
#define GPS_ENABLE_PIN 7
#define GPS_BAUD 9600
#define GPS_MAX_WAIT 120
#define CONSOLE_BAUD 115200

#define SOFT_MISO_PIN 12
#define SOFT_MOSI_PIN 11
#define SOFT_SCK_PIN 13
#define SD_CHIP_SELECT_PIN 10

#define CONFIG_VERSION "ir1"
#define CONFIG_START 32

#define DIAGNOSTICS true

// Example settings structure
struct StoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  int interval, wait, timeout;
} mySettings = {
  CONFIG_VERSION,
  // The default values
  BEACON_INTERVAL, GPS_MAX_WAIT, ROCKBLOCK_SENDRECEIVE_TIME
};

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t=0; t<sizeof(mySettings); t++)
      *((char*)&mySettings + t) = EEPROM.read(CONFIG_START + t);
      
     Serial.println("Found config in EEPROM!");
     Serial.print("interval = "); Serial.println(mySettings.interval);
     Serial.print("wait = "); Serial.println(mySettings.wait);
     Serial.print("timeout = "); Serial.println(mySettings.timeout);
   } else {
     Serial.println("No config found in EEPROM");     
   }
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(mySettings); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&mySettings + t));
}

IridiumSBD modem(Serial3, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;

uint8_t inBuffer[200];
char inBufferString[200];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ROCKBLOCK_SLEEP_PIN, OUTPUT);
  pinMode(GPS_ENABLE_PIN, OUTPUT);

  // Start the serial ports
  Serial.begin(CONSOLE_BAUD);
  Serial1.begin(GPS_BAUD);
  Serial3.begin(ROCKBLOCK_BAUD);

  // Load config from memory
  loadConfig();
  
  // Set max Iridium WAIT
  modem.adjustSendReceiveTimeout(mySettings.timeout);
  
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // 0 = direct connect (default), 1 = USB
  
  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1<<6);//enable interrupt mode
  
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);
  
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
}

void writeToSd(char *message) {
  Serial.println("Writing to sdcard");

  SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
  if (!sd.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println("Error opening sdcard");
    return;
  }
  SdFile dataFile;
  if (!dataFile.open("gps.csv", O_WRONLY | O_APPEND)) {
    Serial.println("Error opening file for writing");
    return;
  }
  dataFile.println(message);
  dataFile.flush();
  dataFile.close();

  // let the sdcard do it's thing
  delay(2000);
}

void loop()
{
  bool fixFound = false;
  int err;
  unsigned long loopStartTime = millis();

  digitalWrite(LED_BUILTIN, HIGH);
  
  // Wake up GPS
  Serial.println("Enabling GPS chip...");
  digitalWrite(GPS_ENABLE_PIN, LOW);
  
  // Step 1: Reset TinyGPS++ and begin listening to the GPS
  Serial.println("Beginning to listen for GPS traffic...");
  tinygps = TinyGPSPlus();

  int gpsChars = 0;
  unsigned long lastFixProgress = 0;
  // Step 2: Look for GPS signal for up to 3 minutes
  for (unsigned long now = millis(); !fixFound && millis() - now < GPS_MAX_WAIT * 1000UL;) {
    if (Serial1.available())
    {
      gpsChars++;
      tinygps.encode(Serial1.read());
      fixFound = tinygps.location.isValid() && tinygps.date.isValid() &&
        tinygps.time.isValid() && tinygps.altitude.isValid();
    }
    if (fixFound || millis() - lastFixProgress > 1000) {
      Serial.print(gpsChars);
      Serial.print(',');
      Serial.println(tinygps.satellites.value());
      lastFixProgress = millis();
    }
  }

  Serial.println(fixFound ? F("A GPS fix was found!") : F("No GPS fix was found."));

  char outBuffer[60]; // Always try to keep message short
  if (fixFound)
  {
    snprintf(outBuffer, sizeof(outBuffer),
      "%02d%02d"
      "%02d%02d,"
      "%s,%s,"
      "%d,%d,%d",
      tinygps.date.month(), tinygps.date.day(),
      tinygps.time.hour(), tinygps.time.minute(),
      String(tinygps.location.lat(), 5).c_str(), String(tinygps.location.lng(), 5).c_str(),
      (int) tinygps.altitude.meters(), (int) tinygps.speed.kmph(), (int) tinygps.course.deg());
  }
  else
  {
    snprintf(outBuffer, sizeof(outBuffer), "No Fix");
  }

  Serial.print("Message (");
  Serial.print(strlen(outBuffer));
  Serial.print( " chars): ");
  Serial.println(outBuffer);

  writeToSd(outBuffer);

  // Disable GPS
  Serial.println("Disabling GPS chip...");
  digitalWrite(GPS_ENABLE_PIN, HIGH);

  // Step 3: Start talking to the RockBLOCK and power it up
  Serial.println("Beginning to talk to the RockBLOCK...");
  int modemStatus = modem.begin();
  if (modemStatus != ISBD_SUCCESS) {
    Serial.print("Modem didn't start, status: ");
    Serial.println(modemStatus);
  } else {
    size_t inBufferSize = sizeof(inBuffer);
        
    Serial.print("Transmitting message");
    err = modem.sendReceiveSBDText(outBuffer, inBuffer, inBufferSize);
    
    if (err != ISBD_SUCCESS)
    {
      Serial.print("sendReceiveSBD* failed: error ");
      Serial.println(err);
    }
    else // success!
    {
      Serial.print("Inbound buffer size is ");
      Serial.println(inBufferSize);
      for (int i=0; i<inBufferSize; ++i)
      {
        inBufferString[i] = inBuffer[i];
        Serial.print(inBuffer[i], HEX);
        if (isprint(inBuffer[i]))
        {
          Serial.print("(");
          Serial.write(inBuffer[i]);
          Serial.print(")");
        }
        Serial.print(" ");
      }
      Serial.println();
      
      // Parse the received settings
      int interval,wait,timeout;
      interval = atoi(strtok(inBufferString,","));
      wait = atoi(strtok(NULL,","));
      timeout = atoi(strtok(NULL,","));
      
      if(interval > 600 && interval <= 86400 * 7) {
        mySettings.interval = interval;
      }
      if(wait > 10 && wait <= 600) {
        mySettings.wait = wait;
      }
      if(timeout > 10 && timeout <= 600) {
        mySettings.timeout = timeout;
      }
     
      saveConfig();
      
      Serial.print("Messages remaining to be retrieved: ");
      Serial.println(modem.getWaitingMessageCount());
    }
    modem.sleep();
  }

  // Sleep
  Serial.println("Going to sleep mode for about an hour...");
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);

  for(int i=0;i<mySettings.interval/8;i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    digitalWrite(LED_BUILTIN, LOW);
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
    
  }
  
  Serial.println("Woke up from sleep!");  
}

void digitalInterrupt(){
  //needed for the digital input interrupt
}

ISR(WDT_vect){
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif

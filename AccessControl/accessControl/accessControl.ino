#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

#define RST_PIN         9          // Configurable, see typical pin layout above
#define SS_PIN          10           // Configurable, see typical pin layout above
#define IRQ_PIN         2           // Configurable, depends on hardware

#define BUZZER_PIN  6
#define SERVO_PIN   3

#define TIMEOUT     5 

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

MFRC522::MIFARE_Key key;

Servo door_servo;

bool door_open = false;
byte last_uid[4] = {0,0,0,0};
byte allowed_uid[4] = {0x3B, 0x37, 0x5A, 0x0A}; //UID 
int8_t time_counter = 0;

volatile bool bNewInt = false;
byte regVal = 0x7F;
void activateRec(MFRC522 mfrc522);
void clearInt(MFRC522 mfrc522);
void play_sound_Ok();
void play_sound_Err();


/**
 * Initialize.
 */
void setup() {
  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial);      // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin();          // Init SPI bus
 
  mfrc522.PCD_Init(); // Init MFRC522 card

  /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
  Serial.print(F("Ver: 0x"));
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println(readReg, HEX);

  /* setup the IRQ pin*/
  pinMode(IRQ_PIN, INPUT_PULLUP);

  /*
   * Allow the ... irq to be propagated to the IRQ pin
   * For test purposes propagate the IdleIrq and loAlert
   */
  regVal = 0xA0; //rx irq
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);

  bNewInt = false; //interrupt flag

  /*Activate the interrupt*/
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);

  /*do { //clear a spourious interrupt at start
    ;
  } while (!bNewInt);*/
  bNewInt = false;

  pinMode(BUZZER_PIN, OUTPUT);
  door_servo.attach(SERVO_PIN);
  door_servo.write(0);
  delay(15); 
  play_sound_Err();
  delay(100);
  play_sound_Ok();  

  Serial.println(F("End setup"));
}



/**
 * Main loop.
 */
void loop() {

  if (bNewInt) { //new read interrupt
    bNewInt = false;
    Serial.print(F("Interrupt. "));
    mfrc522.PICC_ReadCardSerial(); //read the tag data
   
    byte *buffer = mfrc522.uid.uidByte;
    if(mfrc522.uid.size < 4){
      Serial.println("Ignore bad uuid");
      return;
    }

    bool is_old = compareUID(buffer, last_uid);
    if(is_old){
      return;
    }
    time_counter = 0; //start timeout
    copyUID(buffer, last_uid);
    
    Serial.print("Found Key: ");
    if(compareUID(buffer, allowed_uid)){   
      Serial.print("access granted!");
      if(door_open){
        door_servo.write(0);
        delay(15);
        door_open = false;
        Serial.println(" closing door.");
      }else{
        door_servo.write(90);
        delay(15);
        door_open = true;
        Serial.println(" opening door.");
      }      
      play_sound_Ok();
    }else{    
      Serial.println("access denied!");
      play_sound_Err();
    }
    // Show some details of the PICC (that is: the tag/card)
    Serial.print(F("Card UID:"));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();

    clearInt(mfrc522);
    mfrc522.PICC_HaltA();
    
  }
  delay(10);
  timer();
}

uint8_t count_10ms;
void timer(){
  if(count_10ms++ >= 100){
    count_10ms=0;
     //reset last seen uid
    if(time_counter < TIMEOUT){
      time_counter++;    
    }
    if(time_counter == TIMEOUT){ //reset after time is up
      memset(last_uid, 0, 4 * sizeof(int)); 
      time_counter++;
    }
    activateRec(mfrc522);
  }
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}
/**
 * MFRC522 interrupt serving routine
 */
void readCard() {
  bNewInt = true;
}

/*
 * The function sending to the MFRC522 the needed commands to activate the reception
 */
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

/*
 * The function to clear the pending interrupt bits after interrupt serving routine
 */
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}

void play_sound_Err(){
  tone(BUZZER_PIN, 500); // Send 1KHz sound signal...
  delay(200);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
  delay(200);        // ...for 1sec
  tone(BUZZER_PIN, 500); // Send 1KHz sound signal...
  delay(200);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
  delay(200); 
  tone(BUZZER_PIN, 500); // Send 1KHz sound signal...
  delay(200);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
}

void play_sound_Ok(){
  tone(BUZZER_PIN, 1000); // Send 1KHz sound signal...
  delay(100);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
  delay(100);        // ...for 1sec
  tone(BUZZER_PIN, 1000); // Send 1KHz sound signal...
  delay(100);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
}

bool compareUID(byte *buffer, byte *compare){
  for(uint8_t i = 0; i < 4; i++){   
    if(buffer[i] != compare[i])
      return false;
  }
  return true;
}

void copyUID(byte *from, byte *to){
  for(uint8_t i = 0; i < 4; i++){
    to[i] = from[i];
  }
}

#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

#define TIMEOUT     6       //Timemout in s after which a card with the same uid is no longer be ignored

#define RST_PIN     9          // Configurable, see typical pin layout above
#define SS_PIN      10         // Configurable, see typical pin layout above

#define BUZZER_PIN  6
#define SERVO_PIN   5

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
Servo door_servo;

bool door_open = false;
byte last_uid[4] = {0,0,0,0};
byte allowed_uid[4] = {0x3B, 0x37, 0x5A, 0x0A}; //UID 
int8_t time_counter = TIMEOUT;


void setup() {
	Serial.begin(9600);		// Initialize serial communications with the PC
	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	mfrc522.PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
	mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details

  pinMode(BUZZER_PIN, OUTPUT);
  door_servo.attach(SERVO_PIN);
  door_servo.write(0);
  delay(15); 
  play_sound_Err();
  delay(100);
  play_sound_Ok();  
  Serial.println("Initialization done!");
}

void loop() {
  delay(500);
  if(time_counter == 0){
    memset(last_uid, 0, 4 * sizeof(int));    
    time_counter = -1;
  }
  if(time_counter > 0){
    time_counter--;
  }

  // skip if no card is found
	if ( !mfrc522.PICC_IsNewCardPresent() ) {
		return;
	}

	// Select one of the cards
	if ( !mfrc522.PICC_ReadCardSerial() ) {
		return;
	}

  byte *buffer = mfrc522.uid.uidByte;
  if(mfrc522.uid.size < 4){
    Serial.println("Ignore bad uuid");
    return;
  }

  bool is_old = compareUID(buffer, last_uid);
  if(is_old){
    return;
  }
  time_counter = TIMEOUT; //start timeout
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
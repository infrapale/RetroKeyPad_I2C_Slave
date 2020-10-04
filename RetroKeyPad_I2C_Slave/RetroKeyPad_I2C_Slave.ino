
#define ROW0_PIN  10
#define ROW1_PIN  11
#define ROW2_PIN  9
#define ROW3_PIN  8
#define ROW4_PIN  7
#define COL0_PIN  6
#define COL1_PIN  5
#define COL2_PIN  4
#define COL3_PIN  3
#define COL4_PIN  2
#define NBR_ROWS  5
#define NBR_COLS  5

#include <TaHa.h>

const uint8_t ROW_PIN[NBR_ROWS] = {ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };  
const uint8_t COL_PIN[NBR_COLS] = {COL0_PIN, COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN };

enum key_states {
  KEY_IDLE,
  KEY_PRESSED,
  KEY_PRESSED_DEBOUNCED,
  KEY_RELEASED,
  KEY_READY
};

TaHa scan_keypad_handle;
boolean Debug = true;
static uint8_t key_matrix[NBR_ROWS][NBR_COLS]; 
static uint8_t key_state[NBR_ROWS][NBR_COLS]; 
static uint8_t key_debounce[NBR_ROWS][NBR_COLS]; 



void ScanKeypad(void){
    uint8_t row;
    uint8_t col;
    char    key_char;

    for (row = 0; row < NBR_ROWS; row++) {
        digitalWrite(ROW_PIN[row], LOW);
    }
    for (row = 0; row < NBR_ROWS; row++) {  
        digitalWrite(ROW_PIN[row], HIGH);
            
        for (col = 0; col < NBR_COLS; col++) {
            if (digitalRead(COL_PIN[col]) == HIGH) {
                if (Debug) {
                    Serial.print("Row: ");
                    Serial.print(row);
                    Serial.print(" Col: ");
                    Serial.print(col);
                    Serial.print (" Key: ");
                    Serial.println(key_matrix[row][col]);
                }
            }
            else {
                switch (key_state) {
                  case key_states.KEY_IDLE:
                      break;
                }
            }
        }
         digitalWrite(ROW_PIN[row], LOW);
    }  
  
}

void setup() {
    uint8_t row;
    uint8_t col;
    char    key_char;

    // wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
    delay(2000);
    while (!Serial); // wait until serial console is open, remove if not tethered to computer
    Serial.begin(9600);
    Serial.println("RetroKeyPad_I2C_Slave Tom Höglund 2020");
    for (row = 0; row < NBR_ROWS; row++) {
        pinMode(ROW_PIN[row], OUTPUT);
    }
    for (col = 0; col < NBR_COLS; col++) {
        pinMode(COL_PIN[col], INPUT);    
    }
    key_char = '0';
    for (row = 0; row < NBR_ROWS; row++) {      
        for (col = 0; col < NBR_COLS; col++) {
            key_matrix[row][col] = key_char;
            key_char++;
        }
    }  
    scan_keypad_handle.set_interval(10,RUN_RECURRING, ScanKeypad);

}

void loop() {
  // put your main code here, to run repeatedly:
  scan_keypad_handle.run();
  // RawScan();
  //digitalWrite(ROW_PIN[0], HIGH);
  delay(100);
}

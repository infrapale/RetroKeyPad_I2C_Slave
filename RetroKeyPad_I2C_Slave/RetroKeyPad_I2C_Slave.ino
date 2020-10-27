/**
 ******************************************************************************
 * @file    
 * @author  Tom Hoglund  infrapale@gmail.com
 *
 * @brief   
 ******************************************************************************
 * @attention
 * 
 ******************************************************************************
 */


#include <Wire.h>
#include <TaHa.h>


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
#define NBR_KEYS  NBR_ROWS * NBR_COLS

#define KEY_BUF_LEN  8
#define KEY_BUF_MASK 0x07

#define RKP_I2C_ADDR             0x18
#define RKP_REQ_KEY_AVAIL        0x04
#define RKP_REQ_GET_KEY          0x05
#define RKP_EVENT_SET_KEY_VECTOR 0x10




const uint8_t ROW_PIN[NBR_ROWS] = {ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };  
const uint8_t COL_PIN[NBR_COLS] = {COL0_PIN, COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN };

enum key_states {
  KEY_STATE_IDLE,
  KEY_STATE_PRESSED,
  KEY_STATE_PRESSED_DEBOUNCED,
  KEY_STATE_RELEASED,
  KEY_STATE_READY
};

TaHa scan_keypad_handle;
TaHa print_key_handle;

boolean Debug = true;
static uint8_t key_matrix[NBR_ROWS][NBR_COLS]; 
static uint8_t key_state[NBR_ROWS][NBR_COLS]; 
static uint8_t key_debounce[NBR_ROWS][NBR_COLS]; 
static uint8_t key_duration[NBR_ROWS][NBR_COLS]; 
static char    key_buf[KEY_BUF_LEN];
static uint8_t key_func_buf[KEY_BUF_LEN];
static uint8_t key_wr_indx;
static uint8_t key_rd_indx;
static uint8_t key_matrix_indx;
static uint8_t reg_addr;

uint32_t  millis1;
uint32_t  millis2;


/**
 * @brief  
 * @param  
 * @param  
 * @param  
 * @retval 
 */
void ScanKeypad(void){
    uint8_t row;
    uint8_t col;
    char    key_char;

    // millis1 = millis;
    
    for (row = 0; row < NBR_ROWS; row++) {
        digitalWrite(ROW_PIN[row], LOW);
    }
    for (row = 0; row < NBR_ROWS; row++) {  
        digitalWrite(ROW_PIN[row], HIGH);
            
        for (col = 0; col < NBR_COLS; col++) {
            if (digitalRead(COL_PIN[col]) == HIGH) {
                if (Debug) {
                    //Serial.print("Row: "); Serial.print(row);
                    //Serial.print(" Col: "); Serial.print(col);
                    //Serial.print (" Key: "); Serial.print(key_matrix[row][col]);
                    //Serial.print (" Key State: "); Serial.println(key_state[row][col]);
                }
                switch (key_state[row][col]) {
                    case KEY_STATE_IDLE:
                        key_state[row][col] = KEY_STATE_PRESSED;
                        key_debounce[row][col] = 2;
                        key_duration[row][col] = 0;
                        break;
                    case KEY_STATE_PRESSED:
                        if (key_debounce[row][col] > 0 ) {  
                            key_debounce[row][col]--;
                            key_duration[row][col]++;
                        }
                        else {
                            key_state[row][col] = KEY_STATE_PRESSED_DEBOUNCED;
                        }

                    
                        break;
                    case KEY_STATE_PRESSED_DEBOUNCED:
                        key_duration[row][col]++;
                        break;
                    case KEY_STATE_RELEASED:
                        break;
                    case KEY_STATE_READY:
                        break;
                }      
            }
            else {
                switch (key_state[row][col]) {
                    case KEY_STATE_IDLE:
                        break;
                    case KEY_STATE_PRESSED:
                        key_state[row][col] = KEY_STATE_IDLE;
                        break;
                    case KEY_STATE_PRESSED_DEBOUNCED:
                        key_state[row][col] = KEY_STATE_IDLE;
                        
                        key_buf[key_wr_indx] = key_matrix[row][col];
                        if (key_duration[row][col] < 50) {
                            key_func_buf[key_wr_indx] = 1;
                        } else {
                            if (key_duration[row][col] < 100) {
                                key_func_buf[key_wr_indx] = 2;
                            } 
                            else 
                            {
                                key_func_buf[key_wr_indx] = 3;
                            } 
                        }
                        
                        // key_func_buf[key_wr_indx] = key_duration[row][col] ;
                        // Serial.print (" Key pressed: ");
                        // Serial.println(key_matrix[row][col]);
                        key_wr_indx = ++key_wr_indx & KEY_BUF_MASK;
                        break;
                }
            }
        }
         digitalWrite(ROW_PIN[row], LOW);
    }  
    //millis2 = millis;
    //Serial.println(millis2-millis1); 
    //Serial.print('.');
}


/**
 * @brief  
 * @param  
 * @param  
 * @param  
 * @retval 
 */
void PrintKey(void){
    if (key_buf[key_rd_indx] != 0x00) {
        Serial.print (" Key from buffer: ");
        Serial.print(key_buf[key_rd_indx]);
        Serial.print (" func =  ");
        Serial.println(key_func_buf[key_rd_indx]);
        key_buf[key_rd_indx] = 0x00;
        key_rd_indx = ++key_rd_indx & KEY_BUF_MASK;
    }  
    else {
        // Serial.println("No data"); 
    }
}

void setup() {
    uint8_t row;
    uint8_t col;
    char    key_char;
    uint8_t *key_vector;

    // wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
    delay(2000);
    while (!Serial); // wait until serial console is open, remove if not tethered to computer
    Serial.begin(9600);
    Serial.println("RetroKeyPad_I2C_Slave Tom Höglund 2020");

    
    Wire.begin(RKP_I2C_ADDR);      // join i2c bus with address 
    Wire.onRequest(RequestEvent);  // register event
    Wire.onReceive(ReceiveEvent);  // register event    



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
            if (key_char == ':') key_char = 'A';
        }
    }  

    for (key_wr_indx = 0;key_wr_indx <  KEY_BUF_LEN; key_wr_indx++){
        key_buf[key_wr_indx] = 0x00;
    }
    key_wr_indx = 0;
    key_rd_indx = 0;

    key_vector = &key_matrix[0][0];
    //*key_vector++ = '@';
    //*key_vector++ = '#';
    //key_vector = key_vector +3;
    //*key_vector++ = '%';
    //*key_vector++ = '?';
    scan_keypad_handle.set_interval(10,RUN_RECURRING, ScanKeypad);
    print_key_handle.set_interval(50,RUN_RECURRING, PrintKey);

}

void loop() {
  // put your main code here, to run repeatedly:
  scan_keypad_handle.run();
  print_key_handle.run();
  // RawScan();
  //digitalWrite(ROW_PIN[0], HIGH);
}

/**
 * @brief  
 * @param  
 * @param  
 * @param  
 * @retval 
 */

void xReceiveEvent(int howMany)
{ 
    uint8_t  cmd;
    uint8_t  *key_vector;
    uint8_t  data;
    
   Serial.println("receive event");
    reg_addr = Wire.read();   // receive byte as a character 
    switch(reg_addr){
        case RKP_EVENT_SET_KEY_VECTOR:
            key_vector = &key_matrix[0][0];
            while(Wire.available())  
            {
                data = Wire.read();   
                *key_vector++ = data;
            }
            break;
        default: 
            while(Wire.available())  
            {
                data = Wire.read(); 
            }
            break;
    }
}

void ReceiveEvent(int howMany)
{ 
    uint8_t event_state;
    uint8_t reg;
    uint8_t idx;
    uint8_t x; 
    uint8_t *ptr;

    Serial.println("receive event");
    event_state = 0; 
    idx = 0;
    ptr = &key_matrix[0][0]; 
    reg = Wire.read();
    switch (reg){
        case RKP_EVENT_SET_KEY_VECTOR:
            while (0 < Wire.available() && idx < NBR_KEYS){
                x = Wire.read(); // read one character from the I2C
                Serial.print(x);
                ptr = &key_matrix[0][0] + idx++;
                *ptr = x;
            }  
            break;         
    }
 
}


/**
 * @brief  
 * @param  
 * @param  
 * @param  
 * @retval 
 */
void RequestEvent()
{
   static char c = '0';
   //Serial.println("request event");
   if (key_buf[key_rd_indx] != 0x00) {
       Serial.print (" Key from buffer: ");
       Serial.println(key_buf[key_rd_indx]);
       Wire.write(key_buf[key_rd_indx]);
       key_buf[key_rd_indx] = 0x00;
       key_rd_indx = ++key_rd_indx & KEY_BUF_MASK;
    } 
    else{
        Wire.write(0);
    } 
    //Wire.endTransmission(); // stop transmitting

   /*
   switch (reg_addr)
   {
      case RKP_REQ_KEY_AVAIL:
          Wire.write("Z");
          break;
      case RKP_REQ_GET_KEY:
          Wire.write("4");
          break;
   }
   */
}
  

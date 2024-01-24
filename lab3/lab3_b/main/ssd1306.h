#ifndef MAIN_SSD1366_H_
#define MAIN_SSD1366_H_

// Following definitions are bollowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

// Control byte //U:Control byte's 6 lsb are 0s 
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80 //U: Co = 1 -> , D/C# = 0 -> command
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00 //U: Co = 0: continous data,  D/C# = 0 -> command
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40 //U: Co = 0: continuous data, D/C# = 1 -> data

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F // U: double byte command, followed this is a 8 bit value
#define OLED_CMD_DISPLAY_RAM            0xA4	// U: display content from RAM
#define OLED_CMD_DISPLAY_ALLON          0xA5	// U: turn all the ledS on
#define OLED_CMD_DISPLAY_NORMAL         0xA6	// U: bit = 1 -> led on; bit = 0 -> led off
#define OLED_CMD_DISPLAY_INVERTED       0xA7	// U: bit = 0 -> led on; bit = 1 -> led off
#define OLED_CMD_DISPLAY_OFF            0xAE	// U: display off (sleep)
#define OLED_CMD_DISPLAY_ON             0xAF	// U: display on (in normal mode)

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD 
												// U: 2 bytes command, followed this is a value to specify what addressing mode is used
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
												// U: 3 bytes command, 2nd byte is the beginning column, 3rd byte is the ending column
												// U: column range to be displayed
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7
												// U: 3 bytes command, 2nd byte is the beginning page, 3rd byte is the ending page
												// U: page range to be displayed

// Hardware Config (pg.31)	// U: should refer to the datasheet page 38,39 for easier visualization
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40	
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1    
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32) // U: these are analog characteristic. beat me up
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80	
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62) // U: i gave up on this
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

#endif /* MAIN_SSD1366_H_ */
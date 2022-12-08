#include <xc.h>
#include <stdint.h>
#include "OLED.h"
#include "i2c_routines.h"
#include "font.h"
#include "logo.h"

// Framebuffer
uint8_t framebuffer[SSD1306_BUFFERSIZE];	// globales Array mit gesamten Displayinhalt.
											// Framebuffer wird in den Funktionen bearbeitet und dann
											// mit fb_show() komplett zur Anzeige gesendet

void init_OLED(void)
{
    // Display off
    OLED_sendCommand(SSD1306_DISPLAYOFF);

    OLED_sendCommand(SSD1306_SETDISPLAYCLOCKDIV);
    OLED_sendCommand(0x80);

    OLED_sendCommand(SSD1306_SETMULTIPLEX);
    OLED_sendCommand(0x3F);
    
    OLED_sendCommand(SSD1306_SETDISPLAYOFFSET);
    OLED_sendCommand(0x00);
    
    OLED_sendCommand(SSD1306_SETSTARTLINE | 0x00);
    
    // We use internal charge pump
    OLED_sendCommand(SSD1306_CHARGEPUMP);
    OLED_sendCommand(0x14);
    
    // Horizontal memory mode
    OLED_sendCommand(SSD1306_MEMORYMODE);
    OLED_sendCommand(0x00);
    
    OLED_sendCommand(SSD1306_SEGREMAP | 0x01);

    OLED_sendCommand(SSD1306_COMSCANDEC);

    OLED_sendCommand(SSD1306_SETCOMPINS);
    OLED_sendCommand(0x12);

    // Max contrast
    OLED_sendCommand(SSD1306_SETCONTRAST);
    OLED_sendCommand(0xCF);

    OLED_sendCommand(SSD1306_SETPRECHARGE);
    OLED_sendCommand(0xF1);

    OLED_sendCommand(SSD1306_SETVCOMDETECT);
    OLED_sendCommand(0x40);

    OLED_sendCommand(SSD1306_DISPLAYALLON_RESUME);

    // Non-inverted display
    OLED_sendCommand(SSD1306_NORMALDISPLAY);

    // Display on
    OLED_sendCommand(SSD1306_DISPLAYON);
}

//*******************************************************************************
// Übertragen eines Befehls an das OLED-Display

void OLED_sendCommand(uint8_t command)
{
    I2C_start(SSD1306_DEFAULT_ADDRESS);
 	I2C_write(0x00);
    I2C_write(command);
 	I2C_stop();
}

//*******************************************************************************
// Invertierung des OLED-Display

void OLED_invert(uint8_t inverted)
{
    if (inverted){
        OLED_sendCommand(SSD1306_INVERTDISPLAY);
    }
    else{
        OLED_sendCommand(SSD1306_NORMALDISPLAY);
    }
}

//*******************************************************************************
// Übertragen des Framebuffers an das OLED-Display

void OLED_sendFramebuffer(uint8_t *framebuffer)
{
    uint8_t packet = 0;
	uint8_t packet_byte = 0;
	
	OLED_sendCommand(SSD1306_COLUMNADDR);
    OLED_sendCommand(0x00);
    OLED_sendCommand(0x7F);

    OLED_sendCommand(SSD1306_PAGEADDR);
    OLED_sendCommand(0x00);
    OLED_sendCommand(0x07);

    // We have to send the framebuffer as 16 bytes packets
    // Our framebuffer is 1024 bytes long, 1024/16 = 64
    // We have to send 64 packets
    
    for (packet = 0; packet < 64; packet++){
		I2C_start(SSD1306_DEFAULT_ADDRESS);
        
        I2C_write(0x40);
		
        for (packet_byte = 0; packet_byte < 16; ++packet_byte){
			I2C_write(framebuffer[packet*16+packet_byte]);
        }
        
		I2C_stop();
    }
}

//*******************************************************************************
// Zeichnen eines Pixels in den Framebuffer an die Position (pos_x|pos_y).
// Setzten des Pixels mit pixel_status = 1, loeschen mit pixel_status = 0

void fb_drawPixel(uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status)
{
    if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT){
        return;
    }

    if (pixel_status){
        framebuffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
    }
    else{
        framebuffer[pos_x+(pos_y/8)*SSD1306_WIDTH] &= ~(1 << (pos_y&7));
    }
}

//*******************************************************************************
// Zeichnen einer vertikalen Linie in den Framebuffer von (x,y) der Laenge 
// length.

void fb_drawVLine(uint8_t x, uint8_t y, uint8_t length)
{
    uint8_t i = 0;
	
	for (i = 0; i < length; ++i){
        fb_drawPixel(x, i+y, 1);
    }
}

//*******************************************************************************
// Zeichnen einer horizontalen Linie in den Framebuffer von (x|y) der Laenge
// length.

void fb_drawHLine(uint8_t x, uint8_t y, uint8_t length)
{
    uint8_t i = 0;
	
	for (i = 0; i < length; ++i){
        fb_drawPixel(i+x, y, 1);
    }
}

//*******************************************************************************
// Zeichnen eines Rechtecks in den Framebuffer von (x1|y1) zu (x2|y2).
// Das Rechteck kann gefuellt werden (1 = fill) oder leer sein (0 = fill).

void fb_drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill)
{
    uint8_t length = x2 - x1 + 1;
    uint8_t height = y2 - y1;
	int x = 0 , y = 0;
        
    if (!fill){
        fb_drawHLine(x1, y1, length);
        fb_drawHLine(x1, y2, length);
        fb_drawVLine(x1, y1, height);
        fb_drawVLine(x2, y1, height);
    }
    else{
        for (x = 0; x < length; ++x){
            for (y = 0; y <= height; ++y){
                fb_drawPixel(x1+x, y+y1, 1);
            }
        }
    }
}

//*******************************************************************************
// Löschen des Framebuffers

void fb_clear()
{
    uint16_t buffer_location = 0;
	
	for (buffer_location = 0; buffer_location < SSD1306_BUFFERSIZE; buffer_location++){
        framebuffer[buffer_location] = 0x00;
    }
}

//*******************************************************************************
// Invertieren des Framebuffers

void fb_invert(uint8_t status)
{
    OLED_invert(status);
}

//*******************************************************************************
// Darstellen des Framebuffers auf dem OLED-Display

void fb_show()
{
    OLED_sendFramebuffer(framebuffer);
}

void fb_show_bmp(uint8_t *pBmp)
{
    OLED_sendFramebuffer(pBmp);
}

//*******************************************************************************
// Zeichnen eines Zeichens an Position (x,y) - Diese Funktion kann nur indirekt
// mit Hilfe der Funktion fb_draw_string aufgerufen werden, da diese den 
// entsprechenden Index fuer den font-Vector berechnet!

void fb_draw_char (uint16_t x, uint16_t y, uint16_t fIndex)
{
    uint16_t bufIndex = (y << 7) + x;
    uint8_t j;
    
    for(j=0; j < font[fIndex]; j++){
        framebuffer[bufIndex + j] = font[fIndex + j + 1];
    }
}

//*******************************************************************************
// Zeichnen einer Zeichenkette ab Position (x,y)
// x: Startpixel 0...(127-Zeichenbreite)
// y: Zeile 0...7 (auch bei fb_draw_string_big)

void fb_draw_string (uint16_t x, uint16_t y, const char *pS)
{    
    uint16_t lIndex, k;
    
    while(*pS){
        // index the width information of character <c> 
        lIndex = 0;
        for(k=0; k < (*pS - ' '); k++){
            lIndex += (font[lIndex]) + 1;
        }
        
        // draw character
        fb_draw_char(x, y, lIndex);
        
        // move the cursor forward for the next character
        x += font[lIndex] + 1;
        
        // next charachter
        pS++;
    }
}

void fb_draw_string_big (uint16_t x, uint16_t y, const char *pS)
{
    uint8_t k;

    while(*pS){
        for(k=0; k<10; k++){
            framebuffer[( y    << 7) + x + k] = font2[*pS - ' '][k*2  ];
            framebuffer[((y+1) << 7) + x + k] = font2[*pS - ' '][k*2+1];
		}
        
        x += 10;
        
        // next charachter
        pS++;
    }    
}


//*******************************************************************************
void fb_draw_BMP (uint16_t x, uint16_t y)
{
    uint8_t k;
    for(k=0; k<10; k++){
		framebuffer[( y    << 7) + x + k] = FH_logo[0][k*2  ];
		framebuffer[((y+1) << 7) + x + k] = FH_logo[0][k*2+1];
    }
    
    x += 10;

    for(k=0; k<10; k++){
		framebuffer[( y    << 7) + x + k] = FH_logo[1][k*2  ];
		framebuffer[((y+1) << 7) + x + k] = FH_logo[1][k*2+1];
    }

 }

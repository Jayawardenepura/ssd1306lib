/* MIT License
 * 
 * Copyright 2018, Tymofii Khodniev <thodnev @ github>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 */

#include "oled.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdfix.h>

#if !defined(OLED_NO_I2C)
/***** I2C-related logic *****/
uint8_t OLED_cmdbuffer[OLED_CMDBUFFER_LEN];

static uint8_t _i2c_cmd_init[] = {
	0x80, 0x8D, 0x80, 0x14	/* Enable charge pump	 */
	,0x80, 0xAF		/* Display on	      	 */
	,0x80, 0x81, 0x80, 0xFF /* Set brightness to 255 */
	,0x80, 0xA7		/* Enable inversion 	 */
};

static uint8_t _i2c_cmd_setpage[] = {
	0x80, 0x00, 0x80, 0x10, /* Set column cursor to 0 */
	0x80, 0xB0 /* Last nibble in 0xB0 defines page (0xB0..0xB7) */
};

static uint8_t _i2c_cmd_setbrightness[] = {
	0x80, 0x81, 0x80, 0xFF  /* Last byte is brightness level (0..255) */
};

static uint8_t _i2c_cmd_dataprefix[] = {0x40};

static uint8_t i2c_devaddr;
static uint8_t *i2c_prefix_ptr;
static uint8_t *i2c_prefix_count;
static uint8_t *i2c_data_ptr;
static uint16_t i2c_data_count;
static bool i2c_is_fastfail;
static void (*i2c_callback)(void *); /* called after transaction finish */
static void *i2c_callback_args;

/* States used in ISR FSM */
enum I2C_State_e {
	I2C_STATE_IDLE = 0,
	I2C_STATE_STOP,
	I2C_STATE_SLAVEADDR,
	I2C_STATE_WRITEPREFIX,
	I2C_STATE_WRITEBYTE
};
static enum I2C_State_e i2c_state = I2C_STATE_IDLE;


static void I2C_init(uint32_t hz_freq)
{
	i2c_state = I2C_STATE_IDLE;
	/* Enable the Two Wire Interface module */
	power_twi_enable();

	/* Select TWBR and TWPS based on frequency. Quite tricky, the main point */
	/* is that prescaler is a pow(4, TWPS)				 	 */
	/* TWBR * TWPS_prescaler value */
	uint32_t twbr = F_CPU / (2 * hz_freq) - 8;
	uint8_t twps;
	for (twps = 0; twps < 4; twps++) {
		if (twbr <= 255)
			break;
		twbr /= 4;
	}

	TWBR = (uint8_t)twbr;
	TWSR = (TWSR & 0xFC) | (twps & 0x03);

	TWCR = (1 << TWEN) | (1 << TWIE);
}


bool OLED_i2c_tx_shed(uint8_t addr, uint8_t *prefix, uint8_t prefix_len, uint8_t *bytes, uint16_t bytes_len, 
		      void (*end_cbk)(void *), void *cbk_args, bool fastfail)
{
	bool ret = false;
	/* No interrupts can occur while this block is executed */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (i2c_state == I2C_STATE_IDLE) {
			i2c_prefix_ptr = prefix;
			i2c_prefix_count = prefix_len;
			i2c_data_ptr = bytes;
			i2c_data_count = bytes_len;
			i2c_is_fastfail = fastfail;
			i2c_callback = end_cbk;
			i2c_callback_args = cbk_args;
			/* Send START signal and initiating new transaction */
			i2c_state = I2C_STATE_SLAVEADDR;
			i2c_devaddr = (addr << 1);
			TWCR |= (1 << TWSTA) | (1 << TWINT);
			ret = true;
		}
	}
	return ret;
}


ISR(TWI_vect, ISR_BLOCK)
{
	switch(i2c_state) {
	case(I2C_STATE_IDLE):
	case(I2C_STATE_STOP):
		/* transfer stop and go to IDLE*/
		/* signal with callback that transaction is over */
		TWCR |= (1 << TWSTO) | (1 << TWINT);
		i2c_state = I2C_STATE_IDLE;
		(*i2c_callback)(i2c_callback_args);
		break;
	case(I2C_STATE_SLAVEADDR):
		// load value
		TWDR = i2c_devaddr;
		TWCR = (TWCR & ~(1 << TWSTA)) | (1 << TWINT);
		if ((NULL == i2c_prefix_ptr) && (NULL == i2c_data_ptr)) {
			i2c_state = I2C_STATE_STOP;
		} else if (NULL == i2c_prefix_ptr) {
			i2c_state = I2C_STATE_WRITEBYTE;
		} else {
			i2c_state = I2C_STATE_WRITEPREFIX;
		}
		break;
	case(I2C_STATE_WRITEPREFIX):
		// load next byte of prefix
		TWDR = *i2c_prefix_ptr++;
		i2c_prefix_count--;
		TWCR |= (1 << TWINT);
		if (!i2c_prefix_count) {
			i2c_state = (NULL == i2c_data_ptr) ? I2C_STATE_STOP : I2C_STATE_WRITEBYTE;
		}
		break;
	case(I2C_STATE_WRITEBYTE):
		// load next byte
		TWDR = *i2c_data_ptr++;
		i2c_data_count--;
		TWCR |= (1 << TWINT);
		if (!i2c_data_count)
			i2c_state = I2C_STATE_STOP;
		break;
	}
}


/* Callback which essentially does nothing */
static void OLED_cbk_empty(void *args)
{
	// empty callback
}


/* A dummy callback which simply unlocks the oled lock */
static void OLED_cbk_unlock(void *args)
{
	OLED *oled = args;
	OLED_unlock(oled);
}


/* Callbacks which are used to write each page */
static void OLED_cbk_writepage(void *args);
static void OLED_cbk_setwritepage(void *args);
/* Writes page. This is called after OLED_cbk_setwritepage */
static void OLED_cbk_writepage(void *args)
{
	OLED *oled = args;
	if (oled->cur_page >= oled->num_pages) {
		OLED_unlock(oled);
		return;
	}
	uint8_t *lineptr = &oled->frame_buffer[oled->cur_page * (uint16_t)oled->width];
	oled->cur_page++;
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_dataprefix, OLED_ARR_SIZE(_i2c_cmd_dataprefix), 
				lineptr, oled->width,
				&OLED_cbk_setwritepage, oled, true)) {
		// nop
	}
}

/* Sets page index and calls OLED_cbk_writepage via callback */
static void OLED_cbk_setwritepage(void *args)
{
	OLED *oled = args;
	_i2c_cmd_setpage[OLED_ARR_SIZE(_i2c_cmd_setpage) - 1] = 0xB0 | oled->cur_page;
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_setpage, 
                                OLED_ARR_SIZE(_i2c_cmd_setpage), NULL, 0,
				&OLED_cbk_writepage, oled, true)) {
		// nop
	}
}



void OLED_cmd_setbrightness(OLED *oled, uint8_t level)
{
	_i2c_cmd_setbrightness[OLED_ARR_SIZE(_i2c_cmd_setbrightness) - 1] = level;
	OLED_spinlock(oled);
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_setbrightness, 
                                OLED_ARR_SIZE(_i2c_cmd_setbrightness), NULL, 0,
				&OLED_cbk_unlock, oled, true)) {
		// nop
	}
}


void OLED_refresh(OLED *oled)
{
	OLED_spinlock(oled);
	/* Code below is executed under lock */
	oled->cur_page = 0;
	OLED_cbk_setwritepage(oled);
	/* Lock is unlocked after series of callbacks, in the last one */
}
#endif // OLED_NO_I2C


/***** Display-related logic *****/
OLED_err __OLED_init(OLED *oled, uint8_t width, uint8_t height, uint8_t *frame_buffer, uint32_t i2c_freq_hz, uint8_t i2c_addr)
{
	oled->width = width;
	oled->height = height;
	oled->frame_buffer = frame_buffer;
	oled->busy_lock = 1;	/* Initially: 1 - unlocked */

	OLED_I2CWRAP(
		oled->i2c_addr = i2c_addr;
		oled->cur_page = 0;
		oled->num_pages = 8;

		I2C_init(i2c_freq_hz);
		
		if (!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_init, OLED_ARR_SIZE(_i2c_cmd_init),
				      NULL, 0, OLED_cbk_empty, NULL, true)) {
			return OLED_EBUSY;
		}
	) // OLED_I2CWRAP

	return OLED_EOK;
}


OLED_err OLED_put_pixel(OLED *oled, uint8_t x, uint8_t y, bool pixel_state)
{
	if ((x >= oled->width) || (y >= oled->height))
		return OLED_EBOUNDS;
	OLED_put_pixel_(oled, x, y, pixel_state);	/* Use inline */
	return OLED_EOK;
}


OLED_err OLED_put_rectangle(OLED *oled, uint8_t x_from, uint8_t y_from, uint8_t x_to, uint8_t y_to, enum OLED_params params)
{
	if (params > (OLED_BLACK | OLED_FILL))
		return OLED_EPARAMS;
	bool pixel_color = (OLED_BLACK & params) != 0;
	bool is_fill = (OLED_FILL & params) != 0;

	/* Limit coordinates to display bounds */
	uint8_t size_errors = 0;
	uint8_t w_max = oled->width - 1;
	uint8_t h_max = oled->height - 1;
	if (x_from > w_max) {
		x_from = w_max;
		size_errors++;
	}
	if (x_to > w_max) {
		x_to = w_max;
		size_errors++;
	}
	if (y_from > h_max) {
		y_from = h_max;
		size_errors++;
	}
	if (y_to > h_max) {
		y_to = h_max;
		size_errors++;
	}
	/* If all coordinates are out of bounds */
	if (size_errors >= 4)
		return OLED_EBOUNDS;

	//OLED_WITH_SPINLOCK(oled) {
		/* Normalize coordinates */
		/* start_@ indicates coordinates of upper left corner  */
		/* stop_@ indicates coordinates of bottom right corner */
		uint8_t start_x = x_to < x_from ? x_to : x_from; /* x min */
		uint8_t start_y = y_to < y_from ? y_to : y_from; /* y min */
		uint8_t stop_x = x_to > x_from ? x_to : x_from;  /* x max */
		uint8_t stop_y = y_to > y_from ? y_to : y_from;  /* y max */

		if (is_fill) {
			/* Fill whole area */
			for (uint8_t x = start_x; x <= stop_x; x++) {
				for (uint8_t y = start_y; y <= stop_y; y++) {
					OLED_put_pixel_(oled, x, y, pixel_color);
				}
			}
		} else {
			/* Draw outer frame */
			for (uint8_t x = start_x; x <= stop_x; x++) {
				OLED_put_pixel_(oled, x, start_y, pixel_color);
				OLED_put_pixel_(oled, x, stop_y, pixel_color);
			}
			for (uint8_t y = start_y; y <= stop_y; y++) {
				OLED_put_pixel_(oled, start_x, y, pixel_color);
				OLED_put_pixel_(oled, stop_x, y, pixel_color);
			}
		}
	//}

	return OLED_EOK;
}

/*
 * Fixed point multiplication.
 *
 * Multiply two fixed point numbers in u16,16 (unsigned 0.16) format.
 * Returns result in the same format.
 * Rounds to nearest, ties rounded up.
 */
static uint16_t mul_fix_u16(uint16_t x, uint16_t y)
{
    uint16_t result;
    /* Optimized ASM version. */
    asm volatile(
	    "mul  %B1, %B2\n\t"
	    "movw %A0, r0\n\t"
	    "ldi  r19, 0x80\n\t"
	    "clr  r18\n\t"
	    "mul  %A1, %A2\n\t"
	    "add  r19, r1\n\t"
	    "adc  %A0, r18\n\t"
	    "adc  %B0, r18\n\t"
	    "mul  %B1, %A2\n\t"
	    "add  r19, r0\n\t"
	    "adc  %A0, r1\n\t"
	    "adc  %B0, r18\n\t"
	    "mul  %A1, %B2\n\t"
	    "add  r19, r0\n\t"
	    "adc  %A0, r1\n\t"
	    "adc  %B0, r18\n\t"
	    "clr  r1"
        : "=&r" (result)
        : "r" (x), "r" (y)
        : "r18", "r19"
    );
    return result;
}
 
/*
 * Cheap and rough fixed point multiplication: multiply only the high
 * bytes of the operands, return 16 bit result.
 *
 * For some reason, the equivalent macro compiles to inefficient code.
 * This compiles to 3 instructions (mul a,b; movw res,r0; clr r1).
 */
static uint16_t mul_high_bytes(uint16_t x, uint16_t y)
{
    return (uint8_t)(x >> 8) * (uint8_t)(y >> 8);
}
 
static inline int16_t sin_fix(uint16_t x)
{
    return cos_fix(0xc000 + x);
}

/*
 * Fixed point cos() function: sixth degree polynomial approximation.
 *
 * argument is in units of 2*M_PI/2^16.
 * result is in units of 1/2^14 (range = [-2^14 : 2^14]).
 *
 * Uses the approximation
 *      cos(M_PI/2*x) ~ P(x^2), with
 *      P(u) = (1 - u) * (1 - u * (0.23352 - 0.019531 * u))
 * for x in [0 : 1]. Max error = 9.53e-5
 */
int16_t cos_fix(uint16_t x)
{
    uint16_t y, s;
    uint8_t i = (x >> 8) & 0xc0;  // quadrant information
    x = (x & 0x3fff) << 1;        // .15
    if (i & 0x40) x = FIXED(1, 15) - x;
    x = mul_fix_u16(x, x) << 1;   // .15
    y = FIXED(1, 15) - x;         // .15
    s = FIXED(0.23361, 16) - mul_high_bytes(FIXED(0.019531, 17), x); // .16
    s = FIXED(1, 15) - mul_fix_u16(x, s);  // .15
    s = mul_fix_u16(y, s);        // .14
    return (i == 0x40 || i == 0x80) ? -s : s;
}
 
OLED_err OLED_put_curve(OLED *oled,uint8_t x_left, uint8_t y_left, uint16_t tan_l, uint8_t x_right, uint8_t y_right,uint16_t tan_r,enum OLED_params params)
{

	if (params > (OLED_BLACK | OLED_FILL))
		return OLED_EPARAMS;
	bool pixel_color = (OLED_BLACK & params) != 0;
	bool is_fill = (OLED_FILL & params) != 0;

/* Limit coordinates to display bounds */
	uint8_t size_errors = 0;
	uint8_t w_max = oled->width - 1;
	uint8_t h_max = oled->height - 1;
	if (x_left > w_max) {
		x_left = w_max;
		size_errors++;
	}
	if (x_right > w_max) {
		x_right = w_max;
		size_errors++;
	}
	if (y_left > h_max) {
		y_left = h_max;
		size_errors++;
	}
	if (y_right > h_max) {
		y_right = h_max;
		size_errors++;
	}
	if (size_errors >= 4)
		return OLED_EBOUNDS;

	int8_t x_avrg,y_avrg;
	
/* Calculating Sin,Cos via Fixed-point */
	accum sin_val_l = sin_fix(tan_l);
	accum cos_val_l = cos_fix(tan_l);

	accum sin_val_r = sin_fix(tan_r);
	accum cos_val_r = cos_fix(tan_r);


/* Calculating tangnet via q14.14 Fixed-point. 
	Covert: fixed-point -> accum(like a float) | 16384 = 2^14 */
	accum the_machine_tan_l = (sin_val_l/16384.0)/(cos_val_l/16384.0);
	accum the_machine_tan_r = (sin_val_r/16384.0)/(cos_val_r/16384.0);

/* y_left = 15; */

	x_avrg = ((the_machine_tan_r * x_right) - (the_machine_tan_l * x_left) + y_left - y_right) / (the_machine_tan_r - the_machine_tan_l);
	y_avrg = (the_machine_tan_l * (x_avrg - x_left)) + y_left;
	
/* point of intersection of tangents. You may delete this line */
        OLED_put_pixel_(oled,(int8_t)x_avrg,(int8_t)y_avrg,pixel_color); 
	

/* Bresenham's algorithm see 
	https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm */

	int32_t sx = x_right-x_avrg, sy = y_right-y_avrg;
	/* relative values for checks */
	int32_t xx = x_left-x_avrg, yy = y_left-y_avrg, xy;       
 	/* curvature */  
	double dx, dy, err, cur = xx*sy-yy*sx;                   	


 	/* begin with longer part */ 
	if (sx*(int32_t)sx+sy*(int32_t)sy > xx*xx+yy*yy) {
		/* swap P0 P2 */
		x_right = x_left; x_left = sx+x_avrg; y_right = y_left; y_left = sy+y_avrg; cur = -cur;  
	}  
	if (cur != 0) {                                   
		/* x step direction */
		xx += sx; xx *= sx = x_left < x_right ? 1 : -1;    
		/* y step direction */      
		yy += sy; yy *= sy = y_left < y_right ? 1 : -1;     
		/* differences 2nd degree */    
		xy = 2*xx*yy; xx *= xx; yy *= yy;        
	if (cur*sx*sy < 0) {                          
		xx = -xx; yy = -yy; xy = -xy; cur = -cur;
	}

		 /* differences 1st degree */
		dx = 4.0*sy*cur*(x_avrg-x_left)+xx-xy;          
		dy = 4.0*sx*cur*(y_left-y_avrg)+yy-xy;
			   /* error 1st step */    
		xx += xx; yy += yy; err = dx+dy+xy;             
	do {         

		/* plot curve */                     
		OLED_put_pixel_(oled,x_left,y_left,pixel_color);       
		 /* last pixel -> curve finished */                            
	if (x_left == x_right && y_left == y_right) return; 

		 /* save value for test of y step */
		y_avrg = 2*err < dx;            

		/* x step */	     
	if (2*err > dy) { x_left += sx; dx -= xy; err += dy += yy; }
		/* y step */ 
	if (    y_avrg    ) { y_left += sy; dy -= xy; err += dx += xx; } 
		} while (dy < dx );         
	}
	return OLED_EOK;
}

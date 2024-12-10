/*
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/sync.h"

#include "hardware/structs/bus_ctrl.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "multiplexed_input.pio.h"


#define vga_mode vga_mode_320x240_60

#define PIO_INPUT_PIN_BASE 14
#define CAPTUREDEPTH 2000
#define CAPTUREBYTES (CAPTUREDEPTH*sizeof(uint32_t))


void core1_func();
void dma_handler();
void parsebuf( short );
void p2c_4bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in );


void vga_320200_16_planar(scanvideo_scanline_buffer_t *buffer);
void draw_test_pattern_stlow();

// Simple color bar program, which draws 7 colored bars: red, green, yellow, blow, magenta, cyan, white
// Can be used to check resister DAC correctness.
//
// Note this program also demonstrates running video on core 1, leaving core 0 free. It supports
// user input over USB or UART stdin, although all it does with it is invert the colors when you press SPACE

static semaphore_t video_initted;
static bool invert;
static volatile bool parsetrigger = false;
static volatile uint dma_chan;

// my screenbuffer
//#define X 640
//#define Y 320
#define X 320
#define Y 200
static uint8_t pixels[Y*X]; // max 640*320 = 204800 bytes

uint32_t *capture_buf[2];
static volatile unsigned short bufidx = 0;

static const uint16_t def_palette[256] = {
0x0fff,0x000f,0x00f0,0x00ff,0x0f00,0x0f0f,0x0ff0,0x0bbb,
0x0888,0x000a,0x00a0,0x00aa,0x0a00,0x0a0a,0x0aa0,0x0000,
0x0fff,0x0eee,0x0ddd,0x0ccc,0x0bbb,0x0aaa,0x0999,0x0888,
0x0777,0x0666,0x0555,0x0444,0x0333,0x0222,0x0111,0x0000,
0x000f,0x010f,0x020f,0x030f,0x040f,0x050f,0x060f,0x070f,
0x080f,0x090f,0x0a0f,0x0b0f,0x0c0f,0x0d0f,0x0e0f,0x0f0f,
0x0f0e,0x0f0d,0x0f0c,0x0f0b,0x0f0a,0x0f09,0x0f08,0x0f07,
0x0f06,0x0f05,0x0f04,0x0f03,0x0f02,0x0f01,0x0f00,0x0f10,
0x0f20,0x0f30,0x0f40,0x0f50,0x0f60,0x0f70,0x0f80,0x0f90,
0x0fa0,0x0fb0,0x0fc0,0x0fd0,0x0fe0,0x0ff0,0x0ef0,0x0df0,
0x0cf0,0x0bf0,0x0af0,0x09f0,0x08f0,0x07f0,0x06f0,0x05f0,
0x04f0,0x03f0,0x02f0,0x01f0,0x00f0,0x00f1,0x00f2,0x00f3,
0x00f4,0x00f5,0x00f6,0x00f7,0x00f8,0x00f9,0x00fa,0x00fb,
0x00fc,0x00fd,0x00fe,0x00ff,0x00ef,0x00df,0x00cf,0x00bf,
0x00af,0x009f,0x008f,0x007f,0x006f,0x005f,0x004f,0x003f,
0x002f,0x001f,0x000b,0x010b,0x020b,0x030b,0x040b,0x050b,
0x060b,0x070b,0x080b,0x090b,0x0a0b,0x0b0b,0x0b0a,0x0b09,
0x0b08,0x0b07,0x0b06,0x0b05,0x0b04,0x0b03,0x0b02,0x0b01,
0x0b00,0x0b10,0x0b20,0x0b30,0x0b40,0x0b50,0x0b60,0x0b70,
0x0b80,0x0b90,0x0ba0,0x0bb0,0x0ab0,0x09b0,0x08b0,0x07b0,
0x06b0,0x05b0,0x04b0,0x03b0,0x02b0,0x01b0,0x00b0,0x00b1,
0x00b2,0x00b3,0x00b4,0x00b5,0x00b6,0x00b7,0x00b8,0x00b9,
0x00ba,0x00bb,0x00ab,0x009b,0x008b,0x007b,0x006b,0x005b,
0x004b,0x003b,0x002b,0x001b,0x0007,0x0107,0x0207,0x0307,
0x0407,0x0507,0x0607,0x0707,0x0706,0x0705,0x0704,0x0703,
0x0702,0x0701,0x0700,0x0710,0x0720,0x0730,0x0740,0x0750,
0x0760,0x0770,0x0670,0x0570,0x0470,0x0370,0x0270,0x0170,
0x0070,0x0071,0x0072,0x0073,0x0074,0x0075,0x0076,0x0077,
0x0067,0x0057,0x0047,0x0037,0x0027,0x0017,0x0004,0x0104,
0x0204,0x0304,0x0404,0x0403,0x0402,0x0401,0x0400,0x0410,
0x0420,0x0430,0x0440,0x0340,0x0240,0x0140,0x0040,0x0041,
0x0042,0x0043,0x0044,0x0034,0x0024,0x0014,0x0fff,0x0000
};
uint16_t *palette;

int main(void) {
    stdio_init_all();

    puts("initialising...\n");

    /* DMA */
    capture_buf[0] = malloc(CAPTUREBYTES);
    capture_buf[1] = malloc(CAPTUREBYTES);
    hard_assert(capture_buf[0]);
    hard_assert(capture_buf[1]);

    for( int i = 14 ; i <= 28 ; i++ ) {
        gpio_init(i);
        gpio_set_pulls ( i, true, false);
        gpio_set_dir(i,false); // true is out
    }

    palette = malloc( 256 * sizeof( uint16_t ) );
    assert( palette );

    for( int i = 0 ; i < 256 ; i++ ) {
        palette[i] = def_palette[i];
    }

    draw_test_pattern_stlow();
    //sleep_ms(3000);

    // create a semaphore to be posted when video init is complete
    sem_init(&video_initted, 0, 1);

    // launch all the video on core 1, so it isn't affected by USB handling on core 0
    multicore_launch_core1(core1_func);

    // wait for initialization of video to be complete
    sem_acquire_blocking(&video_initted);

#if 0
    puts("Color bars ready, press SPACE to invert...");

    while (true) {
        // prevent tearing when we invert - if you're astute you'll notice this actually causes
        // a fixed tear a number of scanlines from the top. this is caused by pre-buffering of scanlines
        // and is too detailed a topic to fix here.
        scanvideo_wait_for_vblank();
        palette[0] = palette[0] == 0 ? 0xfff : 0;
        int c = getchar_timeout_us(0);
        switch (c) {
            case ' ':
                invert = !invert;
                printf("Inverted: %d\n", invert);
                break;
        }
    }
#else
    PIO pio = pio1;
    uint offset = pio_add_program(pio, &clocked_input_program);
    uint sm = pio_claim_unused_sm(pio, true);


    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This should only be needed if you are pushing things up to
    // >16bits/clk here, i.e. if you need to saturate the bus completely.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
//    channel_config_set_ring (&c, true, 5);


    // Load the clocked_input program, and configure a free state machine
    // to run the program.
    clocked_input_program_init(pio, sm, offset, PIO_INPUT_PIN_BASE);

    bufidx = 0;
    dma_channel_configure(dma_chan, &c,
        capture_buf[bufidx],        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        CAPTUREDEPTH,       // Number of transfers
        true                // Start immediately
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq1_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    printf( "red =   %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0xf, 0x0, 0x0) );
    printf( "green = %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x0, 0xf, 0x0) );
    printf( "blue =  %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x0, 0x0, 0xf) );

    printf("Listening...\n");

    for( ;; ) {
        sleep_ms(20);
        uint8_t *target = (pixels+(X*Y/2));
        p2c_4bpp( target, X*Y, pixels );
    }

#endif



}

static int32_t rxdata[5];

void writemem() {
    uint32_t add;
    uint16_t data;
    bool is8bit = false;
    if( rxdata[3] == -1 ) {
        data = rxdata[4];
        is8bit = true;
        rxdata[2] |= 0x1;
    }
    else if( rxdata[4] == -1 ) {
        data = rxdata[3];
        is8bit = true;
    }
    else {
        data = (rxdata[3]<<8)|rxdata[4];
    }
    add = (rxdata[0] << 16)|(rxdata[1]<<8)|rxdata[2];
    add -= 0x78000;
    add /= 2; // bytes->words
    if( add > sizeof(pixels) )
        add = 0;
        
    uint16_t *pixword = (uint16_t*)pixels;
    pixword[add] = data;

    //printf("Done. 0x%6.6lx = 0x%4.4x %s\n", add, data, is8bit ? " (8 bit only)" : "" );
}

void parsebuf( short idx ) {
/*    
    static uint8_t data[5];
    uint32_t address;
    uint16_t value;

    for( int i = 0 ; i < CAPTUREDEPTH ; i++ ) {

    }
    */
    palette[255] = rand() & 0x0fff;

    uint32_t* ptr = capture_buf[idx];

    for( uint l = 0 ; l < CAPTUREDEPTH ; l++ ) {
        //printf("%lx\n", *ptr );
        uint32_t type = (*ptr)>>12; 
        uint32_t data = (*ptr)&0xff; 
        switch( type ) {
            case(1):
                rxdata[0] = data;
                rxdata[1] = -1;
                rxdata[2] = -1;
                rxdata[3] = -1;
                rxdata[4] = -1;
                break;
            case(2):
            case(3):
            case(4):
                rxdata[type-1] = data;
                break;
            case(5):
                rxdata[4] = data;
                writemem();
                break;
            case(6):
                rxdata[3] = data;
                writemem();
                break;
            default:
                break;        
        }
        ptr++;
    }
}


void dma_handler() {
    short oldbuf = bufidx;
    // Clear the interrupt request.
    dma_channel_acknowledge_irq1( dma_chan );
    // Give the channel a new wave table entry to read from, and re-trigger it
    bufidx = bufidx > 0 ? 0 : 1;
    dma_channel_set_write_addr(dma_chan, capture_buf[bufidx], true);
    //parsetrigger = true;
    parsebuf(oldbuf);
}

void draw_color_bar(scanvideo_scanline_buffer_t *buffer) {
    // figure out 1/32 of the color value
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint32_t primary_color = 1u + (line_num * 7 / vga_mode.height);
    uint32_t color_mask = PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x1f * (primary_color & 1u), 0x1f * ((primary_color >> 1u) & 1u), 0x1f * ((primary_color >> 2u) & 1u));
    uint bar_width = vga_mode.width / 32;

    uint16_t *p = (uint16_t *) buffer->data;

    uint32_t invert_bits = invert ? PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x1f,0x1f,0x1f) : 0;
    for (uint bar = 0; bar < 32; bar++) {
        *p++ = COMPOSABLE_COLOR_RUN;
        uint32_t color = PICO_SCANVIDEO_PIXEL_FROM_RGB5(bar, bar, bar);
        *p++ = (color & color_mask) ^ invert_bits;
        *p++ = bar_width - 3;
    }

    // 32 * 3, so we should be word aligned
    assert(!(3u & (uintptr_t) p));

    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    // end of line with alignment padding
    *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
    *p++ = 0;

    buffer->data_used = ((uint32_t *) p) - buffer->data;
    assert(buffer->data_used < buffer->data_max);

    buffer->status = SCANLINE_OK;
}

void core1_func() {
    // initialize video and interrupts on core 1
    scanvideo_setup(&vga_mode);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);

    while (true) {
#if 0
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        draw_color_bar(scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
#else
       scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        {
            //uint32_t line_begin = time_us_32();
            /*
            if( mode == _BPP4 ) {
                vga_640320_16_planar(scanline_buffer);
            }
            if( mode == _STLOW ) {
                vga_320200_16_planar(scanline_buffer);
            }
            else
                vga_640320_256_chunky(scanline_buffer);
            */
            vga_320200_16_planar(scanline_buffer);

            //uint32_t linediff = time_us_32() - line_begin;
            //linetimes[(scanline_buffer->scanline_id)&0xff] = linediff;
        }
        scanvideo_end_scanline_generation(scanline_buffer);
#endif
    }

}

void vga_320200_16_planar(scanvideo_scanline_buffer_t *buffer) {

    const int LINPIX=320;

    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    line_num -= 20;
    if( line_num < 0 || line_num >= 200 ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
//        *p++ = bufidx  % 2 ? 0x0f0f : 0x00f0; //palette[5];
        *p++ = palette[255];
        *p++ = X - 3;
    }
    else {
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixels+(X*Y/2)+(line_num*X/2)); // 4bpp -- two pix per byte, but second half of framebuffer (for background p2c)

        *p++ = COMPOSABLE_RAW_RUN;
        
        colidx = *src++;
        colidx = 10;
        *p++ = palette[(colidx >> 0)&0xf];
        *p++ = LINPIX - 3;
        *p++ = palette[(colidx >> 4)&0xf];
        *p++ = palette[(colidx >> 8)&0xf];
        *p++ = palette[(colidx >> 12)&0xf];
        *p++ = palette[(colidx >> 16)&0xf];
        *p++ = palette[(colidx >> 20)&0xf];
        *p++ = palette[(colidx >> 24)&0xf];
        *p++ = palette[(colidx >> 28)&0xf];

        for( int i = 8 ; i < LINPIX ; i+=8 ) {
            colidx = *src++;            
            *p++ = palette[(colidx >> 0)&0xf];
            *p++ = palette[(colidx >> 4)&0xf];
            *p++ = palette[(colidx >> 8)&0xf];
            *p++ = palette[(colidx >> 12)&0xf];
            *p++ = palette[(colidx >> 16)&0xf];
            *p++ = palette[(colidx >> 20)&0xf];
            *p++ = palette[(colidx >> 24)&0xf];
            *p++ = palette[(colidx >> 28)&0xf];
        }
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0;
        *p++ = 640-X - 3;
    }


    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    // end of line with alignment padding
    *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
    *p++ = 0;

    buffer->data_used = ((uint32_t *) p) - buffer->data;
    buffer->status = SCANLINE_OK;
}


void draw_test_pattern_stlow() {
    const int bytes_per_line = 160;

    // clear screen to white    
    memset( pixels, 0, bytes_per_line*Y );

    // top half of screen in ST planar mode
    for( int j = 0 ; j < Y/4 ; j++ ) {
        int pixel_in_row = 0;
        for( int i = 0 ; i < X ; i++ ) {
            int pixel_in_row = i;
            int shift = 15-(pixel_in_row%16);

            uint8_t pixel_colour = (16*pixel_in_row/320);

            
            uint16_t *p = (uint16_t*)pixels;
            p += j*bytes_per_line/2 + 4*(pixel_in_row/16);

            *p     |= ( ( pixel_colour>>0 )&0x1 ) << (shift);
            *(p+1) |= ( ( pixel_colour>>1 )&0x1 ) << (shift);
            *(p+2) |= ( ( pixel_colour>>2 )&0x1 ) << (shift);
            *(p+3) |= ( ( pixel_colour>>3 )&0x1 ) << (shift);                    

        }

    }
    for( int j = Y/4 ; j < 2*Y/4 ; j++ ) {
        int pixel_in_row = 0;



        for( int i = 0 ; i < X ; i++ ) {
            int pixel_in_row = i;
            int shift = 15-(pixel_in_row%16);

            uint8_t pixel_colour = (i/2) % 16;
            
            uint16_t *p = (uint16_t*)pixels;
            p += j*bytes_per_line/2 + 4*(pixel_in_row/16);

            *p     |= ( ( pixel_colour>>0 )&0x1 ) << (shift);
            *(p+1) |= ( ( pixel_colour>>1 )&0x1 ) << (shift);
            *(p+2) |= ( ( pixel_colour>>2 )&0x1 ) << (shift);
            *(p+3) |= ( ( pixel_colour>>3 )&0x1 ) << (shift);            

        }

    }
    for( int j = 2*Y/4 ; j < 3*Y/4 ; j++ ) {
        int pixel_in_row = 0;

        for( int i = 0 ; i < X ; i++ ) {
            int pixel_in_row = i;
            int shift = 15-(pixel_in_row%16);

            uint8_t pixel_colour =  ( i / 16 ) % 16;
            
            uint16_t *p = (uint16_t*)pixels;
            p += j*bytes_per_line/2 + 4*(pixel_in_row/16);

            *p     |= ( ( pixel_colour>>0 )&0x1 ) << (shift);
            *(p+1) |= ( ( pixel_colour>>1 )&0x1 ) << (shift);
            *(p+2) |= ( ( pixel_colour>>2 )&0x1 ) << (shift);
            *(p+3) |= ( ( pixel_colour>>3 )&0x1 ) << (shift);            

        }

    }
    for( int j = 3*Y/4 ; j < 4*Y/4 ; j++ ) {
        int pixel_in_row = 0;



        for( int i = 0 ; i < X ; i++ ) {
            int pixel_in_row = i;
            int shift = 15-(pixel_in_row%16);

            uint8_t pixel_colour = i % 16;
            
            uint16_t *p = (uint16_t*)pixels;
            p += j*bytes_per_line/2 + 4*(pixel_in_row/16);

            *p     |= ( ( pixel_colour>>0 )&0x1 ) << (shift);
            *(p+1) |= ( ( pixel_colour>>1 )&0x1 ) << (shift);
            *(p+2) |= ( ( pixel_colour>>2 )&0x1 ) << (shift);
            *(p+3) |= ( ( pixel_colour>>3 )&0x1 ) << (shift);            

        }

    }
}

void p2c_4bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in ) {

    uint8_t pix[16];
    uint16_t *block = (void*)in;
    uint16_t plane[4];

    for( int pixel = 0 ; pixel < pixels_to_convert ; pixel += 16 ) {
        plane[0] = *block++;
        plane[1] = *block++;
        plane[2] = *block++;
        plane[3] = *block++;

        // pixel 1 is the sum of the first bit of each of the (4) words raised by two each time

        for( int i = 0 ; i < 16 ; i++ ) {
            pix[15-i] =    ((( plane[0]>>i) & 0x1 ) << 0) | 
                        ((( plane[1]>>i) & 0x1 ) << 1) |
                        ((( plane[2]>>i) & 0x1 ) << 2) |
                        ((( plane[3]>>i) & 0x1 ) << 3);
        }
//#define SWAP
#ifdef SWAP
        /* byteswap happens here*/
        *(outpix++) = (pix[9] << 4) | pix[8];
        *(outpix++) = (pix[11] << 4) | pix[10];
        *(outpix++) = (pix[13] << 4) | pix[12];
        *(outpix++) = (pix[15] << 4) | pix[14];

        *(outpix++) = (pix[1] << 4) | pix[0];
        *(outpix++) = (pix[3] << 4) | pix[2];
        *(outpix++) = (pix[5] << 4) | pix[4];
        *(outpix++) = (pix[7] << 4) | pix[6];
#else
        *(outpix++) = (pix[1] << 4) | pix[0];
        *(outpix++) = (pix[3] << 4) | pix[2];
        *(outpix++) = (pix[5] << 4) | pix[4];
        *(outpix++) = (pix[7] << 4) | pix[6];

        *(outpix++) = (pix[9] << 4) | pix[8];
        *(outpix++) = (pix[11] << 4) | pix[10];
        *(outpix++) = (pix[13] << 4) | pix[12];
        *(outpix++) = (pix[15] << 4) | pix[14];
#endif
    }    

}

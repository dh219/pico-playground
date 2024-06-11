/*
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>

// Reduce VGA rendering on 12 bits
//target_compile_definitions( ${exec} PRIVATE PICO_SCANVIDEO_COLOR_PIN_BASE=0 PICO_SCANVIDEO_COLOR_PIN_COUNT=12 )
//target_compile_definitions( ${exec} PRIVATE PICO_SCANVIDEO_PIXEL_RSHIFT=0 PICO_SCANVIDEO_PIXEL_GSHIFT=4 PICO_SCANVIDEO_PIXEL_BSHIFT=8 )
//target_compile_definitions( ${exec} PRIVATE PICO_SCANVIDEO_PIXEL_RCOUNT=4 PICO_SCANVIDEO_PIXEL_GCOUNT=4 PICO_SCANVIDEO_PIXEL_BCOUNT=4 )
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/sync.h"

#include "psram_spi.h"
#include "macaw.h"
#include "screendump.h"

#define vga_mode vga_mode_640x480_60
//#define vga_mode vga_mode_320x240_60

void core1_func();

// Simple color bar program, which draws 7 colored bars: red, green, yellow, blow, magenta, cyan, white
// Can be used to check resister DAC correctness.
//
// Note this program also demonstrates running video on core 1, leaving core 0 free. It supports
// user input over USB or UART stdin, although all it does with it is invert the colors when you press SPACE

#define PSRAM 0

enum { _CHUNKY8, _BPP4 };

static short mode = _BPP4;

static semaphore_t video_initted;
static bool invert;
//psram_spi_inst_t* async_spi_inst;
static psram_spi_inst_t psram_spi;
static bool psram_ready;
static uint32_t linetimes[256];

// my screenbuffer
#define X 640
#define Y 320
static uint8_t pixels[Y*X]; // max 640*320 = 204800 bytes
uint8_t l = 0;

static const uint16_t def_palette[256] = {
    0x0fff,0x0fc0,0x00f0,0x0ff0,0x000f,0x0fcf,0x00ff,0x0bbb,
    0x0888,0x0a80,0x00a0,0x0aa0,0x000a,0x0a8a,0x00aa,0x0000,
    0x0fff,0x0eee,0x0ddd,0x0ccc,0x0bbb,0x0aaa,0x0999,0x0888,
    0x0777,0x0666,0x0555,0x0444,0x0333,0x0222,0x0111,0x0000,
    0x0fc0,0x0fc1,0x0fc2,0x0fc3,0x0fc4,0x0fc5,0x0fc6,0x0fc7,
    0x0fc8,0x0fc9,0x0fca,0x0fcb,0x0fcc,0x0fcd,0x0fce,0x0fcf,
    0x0ecf,0x0dcf,0x0ccf,0x0b8f,0x0a8f,0x098f,0x088f,0x074f,
    0x064f,0x054f,0x044f,0x030f,0x020f,0x010f,0x000f,0x001f,
    0x002f,0x003f,0x004f,0x005f,0x006f,0x007f,0x008f,0x009f,
    0x00af,0x00bf,0x00cf,0x00df,0x00ef,0x00ff,0x00fe,0x00fd,
    0x00fc,0x00fb,0x00fa,0x00f9,0x00f8,0x00f7,0x00f6,0x00f5,
    0x00f4,0x00f3,0x00f2,0x00f1,0x00f0,0x01f0,0x02f0,0x03f0,
    0x04f0,0x05f0,0x06f0,0x07f0,0x08f0,0x09f0,0x0af0,0x0bf0,
    0x0cf0,0x0df0,0x0ef0,0x0ff0,0x0fe0,0x0fd0,0x0fc0,0x0ff0,
    0x0fe0,0x0fd0,0x0fc0,0x0ff0,0x0fe0,0x0fd0,0x0fc0,0x0ff0,
    0x0fe0,0x0fd0,0x0b80,0x0b81,0x0b82,0x0b83,0x0b84,0x0b85,
    0x0b86,0x0b87,0x0b88,0x0b89,0x0b8a,0x0b8b,0x0a8b,0x098b,
    0x088b,0x074b,0x064b,0x054b,0x044b,0x030b,0x020b,0x010b,
    0x000b,0x001b,0x002b,0x003b,0x004b,0x005b,0x006b,0x007b,
    0x008b,0x009b,0x00ab,0x00bb,0x00ba,0x00b9,0x00b8,0x00b7,
    0x00b6,0x00b5,0x00b4,0x00b3,0x00b2,0x00b1,0x00b0,0x01b0,
    0x02b0,0x03b0,0x04f0,0x05f0,0x06f0,0x07f0,0x08b0,0x09b0,
    0x0ab0,0x0bb0,0x0ba0,0x0b90,0x0b80,0x0bf0,0x0be0,0x0bd0,
    0x0bc0,0x0bb0,0x0ba0,0x0b90,0x0740,0x0741,0x0742,0x0743,
    0x0744,0x0745,0x0746,0x0747,0x0647,0x0547,0x0447,0x0307,
    0x0207,0x0107,0x0007,0x0017,0x0027,0x0037,0x0047,0x0057,
    0x0067,0x0077,0x0076,0x0075,0x0074,0x0073,0x0072,0x0071,
    0x0070,0x0170,0x0270,0x0370,0x0470,0x0570,0x0670,0x0770,
    0x0760,0x0750,0x0740,0x0770,0x0760,0x0750,0x0440,0x0441,
    0x0442,0x0443,0x0444,0x0304,0x0204,0x0104,0x0004,0x0014,
    0x0024,0x0034,0x0044,0x0043,0x0042,0x0041,0x0040,0x0140,
    0x0240,0x0340,0x0440,0x0470,0x0460,0x0450,0x0fff,0x0000
};



uint16_t *palette;

void makepix4bpp( uint8_t pix[16], uint64_t block ) {
    // returns 16 nybles (8 bytes) for 16 pix translated from a 64 bit planar block

    // pixel 1 is the sum of the first bit of each of the (4) words raised by two each time
    for( int i = 0 ; i < 16 ; i++ ) {
        pix[i] =    (( block>>i) & 0x1  ) | 
                    ((( block>>16+i) & 0x1 ) << 1) |
                    ((( block>>32+i) & 0x1 ) << 2) |
                    ((( block>>48+i) & 0x1 ) << 3);
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
            pix[i] =    ((( plane[0]>>i) & 0x1 ) << 0) | 
                        ((( plane[1]>>i) & 0x1 ) << 1) |
                        ((( plane[2]>>i) & 0x1 ) << 2) |
                        ((( plane[3]>>i) & 0x1 ) << 3);
        }
/*        *(outpix++) = (pix[0] << 4) | pix[1];
        *(outpix++) = (pix[2] << 4) | pix[3];
        *(outpix++) = (pix[4] << 4) | pix[5];
        *(outpix++) = (pix[6] << 4) | pix[7];
        *(outpix++) = (pix[8] << 4) | pix[9];
        *(outpix++) = (pix[10] << 4) | pix[11];
        *(outpix++) = (pix[12] << 4) | pix[13];
        *(outpix++) = (pix[14] << 4) | pix[15];        
*/
        for( int i = 7 ; i >= 0 ; i-- ) {
            *(outpix++) = (pix[2*i] << 4) | pix[2*i+1];
        }
/*
        if( l == 100 && pixel == (0x2e0*2) ) {
            printf("planes[] = %4.4x %4.4x %4.4x %4.4x\n", plane[0], plane[1], plane[2], plane[3] );
            
            printf("pix[0-15] = %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
                pix[0], pix[1], pix[2], pix[3], pix[4], pix[5], pix[6], pix[7], 
                pix[8], pix[9], pix[10], pix[11], pix[12], pix[13], pix[14], pix[15] );
        }*/
    }    

}

void draw_screendump() {
    for( int i = 0 ; i < 640*200/2 ; i+=2) {
        pixels[i] = screen_bin[i+1];
        pixels[i+1] = screen_bin[i];
    }
}

void draw_macaw() {
    for( int i = 0 ; i < X*Y ; i++ )
        pixels[i] = macaw[i];
}

void draw_palette(){
    for( int i = 0 ; i < Y ; i++ ) {
        for( int j = 0 ; j < X ; j++ ) {
            pixels[X*i + j] = (int)( (16.0 * (float)j / (float)X) ) + 16*(int)(16.0 * (float)i / (float)Y);
        }
    }
}

void draw_test_pattern_4bpp() {
    const int bytes_per_line = 320;

    // clear screen to white    
    memset( pixels, 0, bytes_per_line*Y );

    // top half of screen in ST planar mode
    for( int j = 0 ; j < Y/2 ; j++ ) {
        int pixel_in_row = 0;
        for( int i = 0 ; i < X ; i++ ) {
            int pixel_in_row = i;
            uint8_t pixel_colour = (16*pixel_in_row/640);

            uint16_t *p = (uint16_t*)pixels;
            p += j*bytes_per_line/2 + 4*(pixel_in_row/16);

            *p |= ( ( pixel_colour )&0x1 ) << (pixel_in_row%16);
            *(p+1) |= ( ( pixel_colour>>1 )&0x1 ) << (pixel_in_row%16);
            *(p+2) |= ( ( pixel_colour>>2 )&0x1 ) << (pixel_in_row%16);
            *(p+3) |= ( ( pixel_colour>>3 )&0x1 ) << (pixel_in_row%16);            

        }

    }

    // bottom half to screen is copy of top, converted to 4bpp chunky
    uint8_t *target = (pixels+(320*Y/2));
    p2c_4bpp( target, X*Y/2, pixels );

}

int main(void) {
    set_sys_clock_khz(250000, true);

    stdio_init_all();
    psram_ready = false;   

    palette = malloc( 256 * sizeof( uint16_t ) );
    assert( palette );

    for( int i = 0 ; i < 256 ; i++ ) {
        palette[i] = def_palette[i];
    }

    if( mode == _BPP4 )
        //draw_test_pattern_4bpp();
        draw_screendump();
    else
        draw_palette();
    
    sleep_ms(5000);

    // create a semaphore to be posted when video init is complete
    sem_init(&video_initted, 0, 1);

    // launch all the video on core 1, so it isn't affected by USB handling on core 0
    multicore_launch_core1(core1_func);

    printf( "red =   %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0xf, 0x0, 0x0) );
    printf( "green = %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x0, 0xf, 0x0) );
    printf( "blue =  %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x0, 0x0, 0xf) );


#if PSRAM
    puts("Initialising PSRAM...");
    psram_spi = psram_spi_init(pio1, -1);
    puts("PSRAM init complete.");
    psram_ready = true;

    // **************** 16 bits testing ****************

    uint32_t sz = 1280;
    const char *sizestr = "1280 bytes";
    uint32_t psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (sz); addr += 2) {
        psram_write16(&psram_spi, addr, (((addr + 1) & 0xFF) << 8) | (addr & 0xFF));
    }
    uint32_t psram_elapsed = time_us_32() - psram_begin;
    float psram_speed = sz / psram_elapsed;
    printf("16 bit: PSRAM write %s in %d us, %.1f MB/s\n", sizestr, psram_elapsed, (float)psram_speed);

    psram_begin = time_us_32();
    for (uint32_t addr = 0; addr < (sz); addr += 2) {
        uint16_t result = psram_read16(&psram_spi, addr);
        if ((uint16_t)(
                (((addr + 1) & 0xFF) << 8) |
                (addr & 0xFF)) != result
        ) {
            printf("PSRAM failure at address %x (%x != %x) ", addr, (
                (((addr + 1) & 0xFF) << 8) |
                (addr & 0xFF)), result
            );
            return 1;
        }
    }
    psram_elapsed = (time_us_32() - psram_begin);
    psram_speed = sz / psram_elapsed;
    printf("16 bit: PSRAM read %s in %d us, %.1f MB/s\n", sizestr, psram_elapsed, (float)psram_speed);
#endif

    // wait for initialization of video to be complete
    sem_acquire_blocking(&video_initted);

    puts("Color bars ready, press SPACE to invert...");


    while (true) {
        // prevent tearing when we invert - if you're astute you'll notice this actually causes
        // a fixed tear a number of scanlines from the top. this is caused by pre-buffering of scanlines
        // and is too detailed a topic to fix here.

        if( mode == _BPP4 ) {
            // bottom half to screen is copy of top, converted to 4bpp chunky
            uint8_t *target = (pixels+(320*Y/2));
            p2c_4bpp( target, X*Y/2, pixels );
        }

        scanvideo_wait_for_vblank();

        int c = getchar_timeout_us(0);
        switch (c) {
            case ' ':
                invert = !invert;
                printf("Mode: %d\n", invert);
                if(invert)
                    draw_macaw();
                else if( mode == _BPP4 )
                    //draw_test_pattern_4bpp();
                    draw_screendump();
                else
                    draw_palette();
                break;
        }

        l++;
        if( l == 100 ) {
            float avgt = 0;
            for( int i = 0 ; i < 256 ; i++ ) {
                avgt += linetimes[i];
            }
            avgt /= 256.0;
            printf("Average line time=%.3f us\n", avgt );

            /*
            uint32_t offset = 0x1400;
            printf("pixel[%x]=%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
                offset,
                pixels[offset+0],
                pixels[offset+1],
                pixels[offset+2],
                pixels[offset+3],
                pixels[offset+4],
                pixels[offset+5],
                pixels[offset+6],
                pixels[offset+7],
                pixels[offset+8],
                pixels[offset+9],
                pixels[offset+10],
                pixels[offset+11],
                pixels[offset+12],
                pixels[offset+13],
                pixels[offset+14],
                pixels[offset+15]
            );

            offset = offset+320*Y/2;
            printf("pixel[%x]=%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
                offset,
                pixels[offset+0],
                pixels[offset+1],
                pixels[offset+2],
                pixels[offset+3],
                pixels[offset+4],
                pixels[offset+5],
                pixels[offset+6],
                pixels[offset+7],
                pixels[offset+8],
                pixels[offset+9],
                pixels[offset+10],
                pixels[offset+11],
                pixels[offset+12],
                pixels[offset+13],
                pixels[offset+14],
                pixels[offset+15]
            );
            */
        }

    }
}

void draw_color_bar(scanvideo_scanline_buffer_t *buffer) {
    // figure out 1/32 of the color value
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint32_t primary_color = 1u + (line_num * 7 / vga_mode.height);
    uint32_t color_mask = PICO_SCANVIDEO_PIXEL_FROM_RGB5(
        0xf * (primary_color & 1u), 
        0xf * ((primary_color >> 1u) & 1u), 
        0xf * ((primary_color >> 2u) & 1u)
    );
    uint bar_width = vga_mode.width / 0x10;

    uint16_t *p = (uint16_t *) buffer->data;

//    uint32_t invert_bits = invert ? PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x1f,0x1f,0x1f) : 0;
    uint32_t invert_bits = 0;
    for (uint bar = 0; bar < 0x10; bar++) {
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

void get_line_from_psram(scanvideo_scanline_buffer_t *buffer) {
#if PSRAM
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint32_t addr = 0;
    uint32_t line_width_bytes = 1280;
    uint16_t line_width_pix = 640;

    uint16_t block[line_width_bytes];

//    printf("Read line %d\n", line_num );

    addr += line_num * line_width_bytes;

    uint16_t *p = (uint16_t *) buffer->data;
    *p++ = COMPOSABLE_RAW_RUN;
    *p++ = psram_read16(&psram_spi, addr );
    *p++ = line_width_pix-3;
    
    addr += 2;
    for( int col = 1 ; col < line_width_pix ; col ++ ) {
        *p++ = psram_read16(&psram_spi, addr );
        addr+=2;
    }

    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    // end of line with alignment padding
    *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
    *p++ = 0;

    buffer->data_used = ((uint32_t *) p) - buffer->data;
//    printf("Data used=%ld\n", buffer->data_used );
    assert(buffer->data_used < buffer->data_max);

    buffer->status = SCANLINE_OK;
#endif
}

void vga_640320_256_chunky(scanvideo_scanline_buffer_t *buffer ) {

    #define LINPIX 640

    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;
    
    line_num -= 80;
    if( line_num < 0 || line_num >= 320 ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0;
        *p++ = X - 3;
    }
    else {
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixels+(X*line_num));

        *p++ = COMPOSABLE_RAW_RUN;
        
        colidx = *src++;
        *p++ = palette[colidx & 0xff];
        *p++ = LINPIX - 3;
        *p++ = palette[(colidx >> 8)&0xff];
        *p++ = palette[(colidx >> 16)&0xff];
        *p++ = palette[(colidx >> 24)&0xff];

        for( int i = 4 ; i < LINPIX ; i+=4 ) {
            colidx = *src++;            
            *p++ = palette[colidx & 0xff];
            *p++ = palette[(colidx >> 8)&0xff];
            *p++ = palette[(colidx >> 16)&0xff];
            *p++ = palette[(colidx >> 24)&0xff];
        }
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


void vga_640320_16_planar(scanvideo_scanline_buffer_t *buffer) {

    #define LINPIX 640

    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    line_num -= 80;
    if( line_num < 0 || line_num >= 320 ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0;
        *p++ = X - 3;
    }
    else {
#if 0
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixels+(line_num*X/2)); // 4bpp

        *p++ = COMPOSABLE_RAW_RUN;
        
        colidx = *src++;
        *p++ = palette[colidx & 0xf];
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
            *p++ = palette[colidx & 0xf];
            *p++ = palette[(colidx >> 4)&0xf];
            *p++ = palette[(colidx >> 8)&0xf];
            *p++ = palette[(colidx >> 12)&0xf];
            *p++ = palette[(colidx >> 16)&0xf];
            *p++ = palette[(colidx >> 20)&0xf];
            *p++ = palette[(colidx >> 24)&0xf];
            *p++ = palette[(colidx >> 28)&0xf];
        }
#else
        uint8_t colidx;
        uint8_t *src = (uint8_t*)(pixels+(line_num*X/2)); // 4bpp

        *p++ = COMPOSABLE_RAW_RUN;
        
        colidx = *src++;
        *p++ = palette[(colidx >> 0)&0xf];
        *p++ = LINPIX - 3;
        *p++ = palette[(colidx >> 4)&0xf];

        for( int i = 2 ; i < LINPIX ; i+=2 ) {
            colidx = *src++;            
            *p++ = palette[(colidx >> 0)&0xf];
            *p++ = palette[(colidx >> 4)&0xf];
        }
#endif
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


void core1_func() {

    // initialize video and interrupts on core 1
    scanvideo_setup(&vga_mode);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);

    while (true) {
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        {
            uint32_t line_begin = time_us_32();
            if( mode == _BPP4 )
                vga_640320_16_planar(scanline_buffer);
            else
                vga_640320_256_chunky(scanline_buffer);

            uint32_t linediff = time_us_32() - line_begin;
            linetimes[(scanline_buffer->scanline_id)&0xff] = linediff;
        }
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}

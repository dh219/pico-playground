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

#define vga_mode vga_mode_640x480_60

void core1_func();

// Simple color bar program, which draws 7 colored bars: red, green, yellow, blow, magenta, cyan, white
// Can be used to check resister DAC correctness.
//
// Note this program also demonstrates running video on core 1, leaving core 0 free. It supports
// user input over USB or UART stdin, although all it does with it is invert the colors when you press SPACE

#define PSRAM 0

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

static const uint16_t def_palette[256] = {
    0x0fff,0x0000,0x000f,0x00f0,0x0f00,0x0ff0,0x00ff,0x0f0f,
    0x0aaa,0x0777,0x000a,0x00a0,0x0a00,0x0aa0,0x00aa,0x0a0a,
    0x0fff,0x0ddd,0x0ddd,0x0ccc,0x0aaa,0x0aaa,0x0999,0x0777,
    0x0777,0x0666,0x0444,0x0444,0x0333,0x0111,0x0111,0x0000,
    0x000f,0x010f,0x010f,0x030f,0x040f,0x040f,0x060f,0x070f,
    0x070f,0x090f,0x0a0f,0x0a0f,0x0c0f,0x0d0f,0x0d0f,0x0f0f,
    0x0f0d,0x0f0d,0x0f0c,0x0f0a,0x0f0a,0x0f09,0x0f07,0x0f07,
    0x0f06,0x0f04,0x0f04,0x0f03,0x0f01,0x0f01,0x0f00,0x0f10,
    0x0f10,0x0f30,0x0f40,0x0f40,0x0f60,0x0f70,0x0f70,0x0f90,
    0x0fa0,0x0fa0,0x0fc0,0x0fd0,0x0fd0,0x0ff0,0x0df0,0x0df0,
    0x0cf0,0x0af0,0x0af0,0x09f0,0x07f0,0x07f0,0x06f0,0x04f0,
    0x04f0,0x03f0,0x01f0,0x01f0,0x00f0,0x00f1,0x00f1,0x00f3,
    0x00f4,0x00f4,0x00f6,0x00f7,0x00f7,0x00f9,0x00fa,0x00fa,
    0x00fc,0x00fd,0x00fd,0x00ff,0x00df,0x00df,0x00cf,0x00af,
    0x00af,0x009f,0x007f,0x007f,0x006f,0x004f,0x004f,0x003f,
    0x001f,0x001f,0x000a,0x010a,0x010a,0x030a,0x040a,0x040a,
    0x060a,0x070a,0x070a,0x090a,0x0a0a,0x0a0a,0x0a0a,0x0a09,
    0x0a07,0x0a07,0x0a06,0x0a04,0x0a04,0x0a03,0x0a01,0x0a01,
    0x0a00,0x0a10,0x0a10,0x0a30,0x0a40,0x0a40,0x0a60,0x0a70,
    0x0a70,0x0a90,0x0aa0,0x0aa0,0x0aa0,0x09a0,0x07a0,0x07a0,
    0x06a0,0x04a0,0x04a0,0x03a0,0x01a0,0x01a0,0x00a0,0x00a1,
    0x00a1,0x00a3,0x00a4,0x00a4,0x00a6,0x00a7,0x00a7,0x00a9,
    0x00aa,0x00aa,0x00aa,0x009a,0x007a,0x007a,0x006a,0x004a,
    0x004a,0x003a,0x001a,0x001a,0x0007,0x0107,0x0107,0x0307,
    0x0407,0x0407,0x0607,0x0707,0x0706,0x0704,0x0704,0x0703,
    0x0701,0x0701,0x0700,0x0710,0x0710,0x0730,0x0740,0x0740,
    0x0760,0x0770,0x0670,0x0470,0x0470,0x0370,0x0170,0x0170,
    0x0070,0x0071,0x0071,0x0073,0x0074,0x0074,0x0076,0x0077,
    0x0067,0x0047,0x0047,0x0037,0x0017,0x0017,0x0004,0x0104,
    0x0104,0x0304,0x0404,0x0403,0x0401,0x0401,0x0400,0x0410,
    0x0410,0x0430,0x0440,0x0340,0x0140,0x0140,0x0040,0x0041,
    0x0041,0x0043,0x0044,0x0034,0x0014,0x0014,0x0fff,0x0000        
};



uint16_t *palette;

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

int main(void) {
    set_sys_clock_khz(250000, true);

    stdio_init_all();
    psram_ready = false;   

    palette = malloc( 256 * sizeof( uint16_t ) );
    assert( palette );

    for( int i = 0 ; i < 256 ; i++ ) {
        palette[i] = def_palette[i];
    }

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

    uint8_t l = 0;
    while (true) {
        // prevent tearing when we invert - if you're astute you'll notice this actually causes
        // a fixed tear a number of scanlines from the top. this is caused by pre-buffering of scanlines
        // and is too detailed a topic to fix here.

        scanvideo_wait_for_vblank();

        int c = getchar_timeout_us(0);
        switch (c) {
            case ' ':
                invert = !invert;
                printf("Mode: %d\n", invert);
                if(invert)
                    draw_macaw();
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

void get_line_from_ram(scanvideo_scanline_buffer_t *buffer) {

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

void core1_func() {

    // initialize video and interrupts on core 1
    scanvideo_setup(&vga_mode);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);

    while (true) {
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
#if 0        
        if( invert ) {
            uint32_t line_begin = time_us_32();
//            draw_color_bar(scanline_buffer);
            get_line_from_psram(scanline_buffer);

            uint32_t linediff = time_us_32() - line_begin;
            linetimes[(scanline_buffer->scanline_id)&0xff] = linediff;
        }
        else 
#endif
        {
            
            uint32_t line_begin = time_us_32();
            get_line_from_ram(scanline_buffer);
            uint32_t linediff = time_us_32() - line_begin;
            linetimes[(scanline_buffer->scanline_id)&0xff] = linediff;
        }
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}

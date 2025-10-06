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


#define PIO_INPUT_PIN_BASE 14
#define NUMBUFS 10
#define CAPTUREDEPTH 1024
//#define CAPTUREDEPTH 1600
//#define CAPTUREDEPTH 2800
//#define CAPTUREDEPTH 2048
#define CAPTUREBYTES (CAPTUREDEPTH*sizeof(uint16_t))


void core1_func();
static void dma_handler();
void parsebuf( short );
void p2c_8bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in );
void p2c_4bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in );
void p2c_2bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in );

void volatile (*scanline_renderer)(scanvideo_scanline_buffer_t *buffer);

void vga_320_8p(scanvideo_scanline_buffer_t *buffer);
void vga_320_4p(scanvideo_scanline_buffer_t *buffer);

void vga_640_8p(scanvideo_scanline_buffer_t *buffer);
void vga_640_4p(scanvideo_scanline_buffer_t *buffer);
void vga_640_2p(scanvideo_scanline_buffer_t *buffer);
void vga_640_1p(scanvideo_scanline_buffer_t *buffer);

void draw_test_pattern_stlow();
void clear_screen();

// Simple color bar program, which draws 7 colored bars: red, green, yellow, blow, magenta, cyan, white
// Can be used to check resister DAC correctness.
//
// Note this program also demonstrates running video on core 1, leaving core 0 free. It supports
// user input over USB or UART stdin, although all it does with it is invert the colors when you press SPACE

static semaphore_t video_initted;
static bool invert;
static volatile uint dma_chan[2];
static volatile bool doublebuf = false;
static volatile int rez = 0;
static volatile int mode = 0;

static struct scanvideo_mode *current_mode;

#define QUEUELEN 0x10
static volatile short queueread = 0;
static volatile short queuewrite = 0;
static volatile short parsequeue[QUEUELEN];

#define SCREENHIST 5 // keep last N screen register addresses

struct SCREENTIME {
    uint32_t base;
    absolute_time_t t;
} screentimes[SCREENHIST];

uint32_t screenreg = 0x0;
uint32_t screenbase[2] = {0x78000,0xf20000};

static volatile uint64_t _vbls = 0;

// my screenbuffer
#define MAXX 640
#define MAXY 480
#define MAXDEPTH 8

static short X = 320;
static short Y = 200;
static short DEPTH = 4;
static bool chunky = false;
static bool doubleline;
//static uint8_t pixels[MAXX*MAXY*MAXDEPTH/8]; // ST resolutions are all the same (for now) // max 640*320 = 204800 bytes
static uint8_t pixels[400000]; // ST resolutions are all the same (for now) // max 640*320 = 204800 bytes
static uint8_t *pixout;
static uint8_t *pixin[2];

uint16_t *capture_buf[NUMBUFS];
static volatile unsigned short dmabufidx[2];

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

struct scanvideo_timing vga_timing_640x480_60_local =
{
    .clock_freq = 25000000,

    .h_active = 640,
    .v_active = 480,

    .h_front_porch = 16,
    .h_pulse = 64,
    .h_total = 800,
    .h_sync_polarity = 1,

    .v_front_porch = 1,
    .v_pulse = 2,
    .v_total = 523,
    .v_sync_polarity = 1,

    .enable_clock = 0,
    .clock_polarity = 0,

    .enable_den = 0
};

struct scanvideo_timing vga_timing_640x480_50_local =
{
    .clock_freq = 25000000,

    .h_active = 640,
    .v_active = 480,

    .h_front_porch = 16,
    .h_pulse = 64,
    .h_total = 800,
    .h_sync_polarity = 1,

    .v_front_porch = 1,
    .v_pulse = 107,
    .v_total = 630,
    .v_sync_polarity = 1,

    .enable_clock = 0,
    .clock_polarity = 0,

    .enable_den = 0
};
struct scanvideo_timing vga_timing_pal_local =
{
    .clock_freq = 25000000,

    .h_active = 640,
    .v_active = 256,

    .h_front_porch = 16,
    .h_pulse = 64,
    .h_total = 1600,
    .h_sync_polarity = 1,

    .v_front_porch = 1,
    .v_pulse = 107,
    .v_total = 628,
    .v_sync_polarity = 1,

    .enable_clock = 0,
    .clock_polarity = 0,

    .enable_den = 0
};


extern const struct scanvideo_pio_program video_24mhz_composable;
struct scanvideo_mode  vga_mode_320240 =
{
    .default_timing = &vga_timing_640x480_60_local,
    .pio_program = &video_24mhz_composable,
    .width = 640,
    .height = 480,
    .xscale = 2,
    .yscale = 2,
};
struct scanvideo_mode  vga_mode_640240 =
{
    .default_timing = &vga_timing_640x480_60_local,
    .pio_program = &video_24mhz_composable,
    .width = 640,
    .height = 480,
    .xscale = 1,
    .yscale = 2,
};
struct scanvideo_mode  vga_mode_640480 =
{
    .default_timing = &vga_timing_640x480_60_local,
    .pio_program = &video_24mhz_composable,
    .width = 640,
    .height = 480,
    .xscale = 1,
    .yscale = 1,
};

void setup_pixelpointers( int split ) {
    uint32_t sz = X*Y*DEPTH/8;

    // initial presumtion is no splitting
    pixin[0] = pixels;
    pixin[1] = pixels;
    pixout = pixels;

    if( chunky || DEPTH == 1 ) { // allow double buf, but no need for p2c so we can ignore split
        pixin[0] = pixels;
        pixin[1] = pixels+sz;
        pixout = pixin[0];
    }
    else if( split == 3 ) {
        assert( sz * 3 <= sizeof( pixels ) );
        pixin[0] = pixels;
        pixin[1] = pixels+sz;
        pixout = pixels + 2*sz;
    }
    else if( split == 2 ) {
        assert( sz * 2 <= sizeof( pixels ) );
        pixin[0] = pixels;
        pixin[1] = NULL;
        pixout = pixels+sz;
    }
}

uint nextbuf(){
    static uint lastallocatedbuf = 0;
    uint newbuf;
    newbuf = (lastallocatedbuf + 1) % NUMBUFS;
    lastallocatedbuf = newbuf;
//    printf("Newbuf=%d\n", newbuf);
    return newbuf;
}

void parsecheck() {
    while( parsequeue[queueread] >= 0 ) {
//        printf("Parsebuf=%d\n", parsequeue[queueread] );
        parsebuf(parsequeue[queueread]);
        parsequeue[queueread] = -1;
        queueread = (queueread+1) % QUEUELEN;
    }
}

void setup_resolution(int newrez, int newmode) {
    rez = newrez;
    mode = newmode;

    int bufsplit = 2;
    
    if( rez == 3 && newmode != -1 ) {
        
        X = (newmode & 0x8) ? 640 : 320;
        doubleline = (newmode & 0x100);
        Y = doubleline ? 240 : 480;

        switch( newmode & 0x7 ) {
            case(3):
                DEPTH = 8;
                X = 320;
                Y = 240;
                doubleline = true;
                scanline_renderer = &vga_320_8p;
                break;
            case(2):
                DEPTH = 4;
                scanline_renderer = (X == 320) ? &vga_320_4p : &vga_640_4p;
                break;
            case(1):
                DEPTH = 2;
                scanline_renderer = &vga_640_2p;
                break;
            case(0):
            default:
                chunky = true; // even if false
                scanline_renderer = &vga_640_1p;
                break;
        }
    }
    else {
        switch( rez ) {
            case(0x4):
                X = 640;
                Y = 480;
                DEPTH = 4;
                scanline_renderer = &vga_640_4p;
                break;
            // ST modes
            case(0x2):
                X = 640;
                Y = 400;
                DEPTH = 1;
                scanline_renderer = &vga_640_1p;
                bufsplit = 2;
                break;
            case(0x1):
                X = 640;
                Y = 200;
                DEPTH = 2;
                scanline_renderer = &vga_640_2p;
                bufsplit = 3;
                doubleline = true;
                break;
            case(0x0):
            default:
                X = 320;
                Y = 200;
                DEPTH = 4;
                scanline_renderer = &vga_320_4p;
                bufsplit = 3;
                doubleline = true;
                break;
        }
    }
    setup_pixelpointers(bufsplit);
}

int main(void) {
    stdio_init_all();

    set_sys_clock_khz(250000, true);
//    set_sys_clock_khz(250000, true);

    sleep_ms(4000);

    puts("initialising...\n");

    printf("PICO_SCANVIDEO_COLOR_PIN_BASE: %d\n", PICO_SCANVIDEO_COLOR_PIN_BASE);
    printf("PICO_SCANVIDEO_COLOR_PIN_COUNT: %d\n", PICO_SCANVIDEO_COLOR_PIN_COUNT);
    printf("PICO_SCANVIDEO_PIXEL_RSHIFT: %d\n", PICO_SCANVIDEO_PIXEL_RSHIFT);
    printf("PICO_SCANVIDEO_PIXEL_GSHIFT: %d\n", PICO_SCANVIDEO_PIXEL_GSHIFT);
    printf("PICO_SCANVIDEO_PIXEL_BSHIFT: %d\n", PICO_SCANVIDEO_PIXEL_BSHIFT);
    printf("PICO_SCANVIDEO_PIXEL_RCOUNT: %d\n", PICO_SCANVIDEO_PIXEL_RCOUNT);
    printf("PICO_SCANVIDEO_PIXEL_GCOUNT: %d\n", PICO_SCANVIDEO_PIXEL_GCOUNT);
    printf("PICO_SCANVIDEO_PIXEL_BCOUNT: %d\n", PICO_SCANVIDEO_PIXEL_BCOUNT);
    printf("PICO_SCANVIDEO_SYNC_PIN_BASE: %d\n", PICO_SCANVIDEO_SYNC_PIN_BASE);
  

    for(int i = 0 ; i < QUEUELEN ; i++)
        parsequeue[i] = -1;

    /* DMA */
    for( int i = 0 ; i < NUMBUFS ; i++ ) {
        capture_buf[i] = calloc(CAPTUREBYTES,1);
        hard_assert(capture_buf[i]);
    }

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

    for( int i = 0 ; i < SCREENHIST ; i++ ) {
        screentimes[i].base = screenreg;
        screentimes[i].t = get_absolute_time();
    }


    setup_resolution(rez, -1);
    draw_test_pattern_stlow();

    // create a semaphore to be posted when video init is complete
    sem_init(&video_initted, 0, 1);


    // launch all the video on core 1, so it isn't affected by USB handling on core 0
    multicore_launch_core1(core1_func);

    // wait for initialization of video to be complete
    sem_acquire_blocking(&video_initted);

    //sleep_ms(5000);

    PIO pio = pio1;
    uint offset = pio_add_program(pio, &clocked_input_program);
    uint sm = pio_claim_unused_sm(pio, true);

    // Load the clocked_input program, and configure a free state machine
    // to run the program.
    clocked_input_program_init(pio, sm, offset, PIO_INPUT_PIN_BASE);


    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This should only be needed if you are pushing things up to
    // >16bits/clk here, i.e. if you need to saturate the bus completely.
#define DMAPRIORITY 0
#ifdef DMAPRIORITY
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
#endif

    dma_chan[0] = dma_claim_unused_channel(true);
    dma_chan[1] = dma_claim_unused_channel(true);

    dma_channel_config dma_config[2];

    // channel 1
    dma_config[0] = dma_channel_get_default_config(dma_chan[0]);
    channel_config_set_read_increment(&dma_config[0], false);
    channel_config_set_write_increment(&dma_config[0], true);
    channel_config_set_dreq(&dma_config[0], pio_get_dreq(pio, sm, false));
//    channel_config_set_ring (&dma_config[0], true, 5);
    channel_config_set_chain_to(&dma_config[0], dma_chan[1]);
    // Tell the DMA to raise IRQ line 1 when the channel finishes a block
    channel_config_set_transfer_data_size(&dma_config[0], DMA_SIZE_16);
    dma_channel_set_irq1_enabled(dma_chan[0], true);

    dmabufidx[0] = nextbuf();
    dma_channel_configure(dma_chan[0], &dma_config[0],
        capture_buf[dmabufidx[0]],        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        CAPTUREDEPTH,       // Number of transfers
        true                // Start immediately
    );


    // channel 2
    dma_config[1] = dma_channel_get_default_config(dma_chan[1]);
    channel_config_set_read_increment(&dma_config[1], false);
    channel_config_set_write_increment(&dma_config[1], true);
    channel_config_set_dreq(&dma_config[1], pio_get_dreq(pio, sm, false));
//    channel_config_set_ring (&dma_config[1], true, 5);
    channel_config_set_chain_to(&dma_config[1], dma_chan[0]);
    channel_config_set_transfer_data_size(&dma_config[1], DMA_SIZE_16);
    dma_channel_set_irq1_enabled(dma_chan[1], true);

    dmabufidx[1] = nextbuf();
    dma_channel_configure(dma_chan[1], &dma_config[1],
        capture_buf[dmabufidx[1]],        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        CAPTUREDEPTH,       // Number of transfers
        false               // Start immediately
    );

    // Configure the processor to run dma_handler() when DMA IRQ 1 is asserted
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);


    printf( "red =   %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0xf, 0x0, 0x0) );
    printf( "green = %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x0, 0xf, 0x0) );
    printf( "blue =  %4.4x\n", PICO_SCANVIDEO_PIXEL_FROM_RGB5(0x0, 0x0, 0xf) );

    printf("Listening...\n");

    uint32_t oldreg = screenreg;
    uint64_t oldvbl = _vbls;
    //screenbase[1] = screenbase[0];
    //screenbase[0] = screenreg;

    for(int i = 0 ; i < QUEUELEN ; i++)
        parsequeue[i] = -1;

    for( ;; ) {

        parsecheck();

        if( _vbls < oldvbl + 1 )
            continue;
        oldvbl = _vbls;

        if( !chunky )  {
            uint8_t *src = pixin[0];
            switch( DEPTH ) {
                case(0): // chunky
                case(1):
                    break;
                case(2):
                    p2c_2bpp( pixout, X*Y, src );
                    break;
                case(8):
                    p2c_8bpp( pixout, X*Y, src );
                    break;
                case(4):
                default:
                    p2c_4bpp( pixout, X*Y, src );
                    break;
            }
        }

#if 0
        if( screenreg != oldreg ) {
            for( int i = SCREENHIST-1 ; i >= 1 ; i-- )
                screentimes[i] = screentimes[i-1];
            screentimes[0].base = screenreg;
            screentimes[0].t = get_absolute_time();
            //if( !doublebuf )
            //   clear_screen();
        }
#if 0
        int64_t us = absolute_time_diff_us( screentimes[SCREENHIST-1].t, get_absolute_time() );
        if( us < 5e6 ) // more than SCREENHIST screen changes in N seconds
            doublebuf = true;
        else
            doublebuf = false;
#endif

        screenbase[0]  = screentimes[doublebuf?1:0].base;
        oldreg = screenreg;
#endif
    }
}

static int32_t rxdata[5];

#define STVIDHIGH   0xff8200
#define STVIDMID    0xff8202
#define STVIDLOW    0xff820c

#define STPALETTE   0xff8240
#define STPALMASK   0xffffc0

#define FALPALETTE  0xff9800
#define FALPALMASK  0xfffc00

#define STRESSET    0xff8260
#define DDB1REG     0xf1ddb0

#define DDB1REGSCR  0x000300


#define DDB1REGX    0x000302
#define CMD_CHUNKY  0x1
#define CMD_BUFFER  0x2


void writemem( short bufinuse ) {
    uint32_t add;
    uint8_t datah;
    uint8_t datal;
    bool high = false;
    bool low = false;

    if( rxdata[3] >= 0 ) {
        datah = rxdata[3];
        high = true;
    }
    if( rxdata[4] >= 0 ) {
        datal = rxdata[4];
        low = true;
    }

    add = (rxdata[0] << 16)|(rxdata[1]<<8)|rxdata[2];

    /*
    if( add < X*Y*DEPTH/8 ) {
        uint8_t *dst = pixin[0];
        dst[add] = datah;
        dst[add+1] = datal;
    }
    return;*/
/*
    if( add > 0x78600 && add < 0x80000 
//        && ( ( datah != 0x0 && datah != 0x55 & datah != 0xaa ) ||  ( datal != 0x0 && datal != 0x55 & datal != 0xaa ) )
//        && ( ( datah != 0xff ) ||  ( datal != 0x0 ) )
    )
        printf("%p = %x %x\n", add, datah, datal );
*/

    uint32_t screen_offset;
    if( add >= screenbase[0] && (screen_offset = add - screenbase[0]) < X*Y * DEPTH/8 ) // within the screen
    {
        uint8_t *dst = pixin[0];
        if( high )
            dst[screen_offset] = datah;
        if( low )
            dst[screen_offset+1] = datal;
        
        return;
    }

/*
    screen_offset = add - screenbase[1];
    if( add >= screenbase[1] && screen_offset < X*Y * DEPTH/8 ) // within the screen
    {
        if( high )
            pixin[1][screen_offset] = datah;
        if( low )
            pixin[1][screen_offset+1] = datal;
        return;
    }
    */
    if( rxdata[0] < 0xf0 )
        return;



#ifdef STE
    // breaks on my -FM. perhaps normal given this is an STE register. Assumed it wasn't used.
    if( add == STVIDLOW ) {
        screen = true;
        screenreg &= 0xffff00;
        screenreg |= (uint32_t)datal;
        return;
    }
    else
#endif
    if( add == STVIDMID ) {
        screenreg &= 0xff00ff;
        screenreg |= ((uint32_t)datal)<<8;
        return;
    }
    else if( add == STVIDHIGH ) {
        screenreg &= 0x00ffff;
        screenreg |= ((uint32_t)datal)<<16;
        return;
    }
    else if( add == DDB1REGSCR ) {
        screenreg = ((uint32_t)datah)<<16 | ((uint32_t)datal)<<8 ; 
        palette[0] = ~palette[0];
        return;
    }
    else if( (add == STRESSET) ) {
        rez = (datah & 0x7);
        chunky = false;
        setup_resolution(rez,-1);
    }
    else if( (add & STPALMASK) == STPALETTE ) {
        uint32_t index = ( add - STPALETTE )>>1;
        if( low ) {
            uint16_t blue   = ((datal & 0x7) << 1) + ( (datal>>3) & 0x1 );
            uint16_t green  = ((datal >> 3) & 0xe) + ( (datal>>7) & 0x1 );
            palette[index] &= 0x000f;
            palette[index] |= ( (blue << 8) | ( green << 4 ) );
        }
        if( high ) {
            uint16_t red   = ((datah & 0x7) << 1) + ( (datah>>3) & 0x1 );
            palette[index] &= 0x0ff0;
            palette[index] |= ( red );
        }
        return;
    }
    else if( (add & FALPALMASK) == FALPALETTE && high && low ) {
        uint32_t index = ( add - FALPALETTE ) >> 2;
        if( add & 2 ) { // blue
            uint16_t blue  = (datal) & 0xf0;
            palette[index] &= 0x00ff;
            palette[index] |= blue << 4;
        }
        else
        { // red/green
            uint16_t red   = datah >> 4;
            uint16_t green  = datal >> 4;

            palette[index] &= 0x0f00;
            palette[index] |= (green<<4) | red;
        }
    }
    else if( add == DDB1REG && high && low ) {
        int mode = datah << 8 | datal; // byte swap
        setup_resolution(3,mode);
    }
    else if( add == DDB1REGX && high && low ) {
        if( datah & CMD_CHUNKY ) {
            chunky = ( datal > 0 );
            setup_resolution(rez,mode);
        }
        if( datah & CMD_BUFFER && chunky ) {
            if( datal != 0 )
                pixout = pixin[1];
            else
                pixout = pixin[0];
        }
    }
}

void parsebuf( short idx ) {

    uint16_t* ptr = capture_buf[idx];

    uint16_t tmp[CAPTUREDEPTH];
    memcpy( tmp, ptr, CAPTUREBYTES );
    memset( ptr, 0, CAPTUREBYTES );
    ptr = tmp;

    uint16_t type;
    uint16_t data;

    uint16_t oldtype = 0;


    bool startvalid = false;

    for( uint l = 0 ; l < CAPTUREDEPTH ; l++ ) {
/*
        if( l % 16 == 1 )
            printf("\n");
        printf("%8.8x ", (*ptr));
*/  

#if 0
        for( uint hl = 0 ; hl < 2 ; hl++ ) 
        {
            if( hl == 1 ) {
                type = ((*ptr)>>12)&0x7; 
                data = ((*ptr))&0xff; 
            }
            else{
                type = ((*ptr)>>12>>15)&0x7; 
                data = ((*ptr)>>15)&0xff;
            }
#else
        {
            type = ((*ptr)>>12)&0x7; 
            data = ((*ptr))&0xff; 
//            printf("type = %x, data = %x\n", type, data);

#endif            
            /* loop here discarding anything until we get an 01? */
            if( !startvalid && type != 1 ) {
                ptr++;
                continue;
            }
            startvalid = true;

/*
            if( type < oldtype ) { // problem in the order
                printf("Type order fault (%d < %d)\n", type, oldtype );
            }
            oldtype = type;

            printf("%d: %2.2x\n", type, data);
*/
            switch( type ) {
                case(1):
                    rxdata[0] = data;
                    rxdata[1] = -1;
                    rxdata[2] = -1;
                    rxdata[3] = -1;
                    rxdata[4] = -1;
                    break;
                case(2):
                    rxdata[1] = data;
                    break;
                case(3):
                    rxdata[2] = data;
                    break;
                case(4):
                    rxdata[3] = data;
                    break;
                case(5):
                    rxdata[3] = data;
                    writemem(idx);
                    oldtype = 0;
                    break;
                case(6):
                    rxdata[4] = data;
                    writemem(idx);
                    oldtype = 0;
                    break;
                default:
                    break;        
            }
        }
        ptr++;
    }
}

#if 0
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
#else
// Called when one of the DMA buffers is full
static void dma_handler() {
    unsigned short oldbuf;

    // DMA chan 1.
    if (dma_hw->ints1 & 1u << dma_chan[0]) {
        // Clear the interrupt request.
//        dma_hw->ints1 = 1u << dma_chan[0];

        // reset chan 1 write address for next time
        oldbuf = dmabufidx[0];
        dmabufidx[0] = nextbuf();
        dma_channel_set_write_addr(dma_chan[0], capture_buf[dmabufidx[0]], false);
        // handle stuff
        //parsebuf(oldbuf);
        parsequeue[queuewrite] = oldbuf;
        queuewrite = (queuewrite+1) % QUEUELEN;
        dma_channel_acknowledge_irq1( dma_chan[0] );
    }
    // DMA chan 2.
    else if (dma_hw->ints1 & 1u << dma_chan[1]) {
        // Clear the interrupt request.
//        dma_hw->ints1 = 1u << dma_chan[1];

        // reset chan 2 write address for next time
        oldbuf = dmabufidx[1];
        dmabufidx[1] = nextbuf();
        dma_channel_set_write_addr(dma_chan[1], capture_buf[dmabufidx[1]], false);
        // handle stuff
        //parsebuf(oldbuf);
        parsequeue[queuewrite] = oldbuf;
        queuewrite = (queuewrite+1) % QUEUELEN;
        dma_channel_acknowledge_irq1( dma_chan[1] );
  }
}
#endif


void clear_screen() {
    // clear screen to white    
    memset( pixels, 0xff, X*Y*DEPTH/8 );
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

void p2c_8bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in ) {

    uint8_t pix[16];
    uint16_t *block = (void*)in;
    uint16_t plane[8];

    for( int pixel = 0 ; pixel < pixels_to_convert ; pixel += 16 ) {

//        if( pixel % 320  == 0 )
//            parsecheck();

        plane[0] = *block++;
        plane[1] = *block++;
        plane[2] = *block++;
        plane[3] = *block++;
        plane[4] = *block++;
        plane[5] = *block++;
        plane[6] = *block++;
        plane[7] = *block++;

        // pixel 1 is the sum of the first bit of each of the (8) words raised by two each time
        for( int i = 0 ; i < 16 ; i++ ) {
            pix[15-i] =    ((( plane[0]>>i) & 0x1 ) << 0) | 
                        ((( plane[1]>>i) & 0x1 ) << 1) |
                        ((( plane[2]>>i) & 0x1 ) << 2) |
                        ((( plane[3]>>i) & 0x1 ) << 3) |
                        ((( plane[4]>>i) & 0x1 ) << 4) |
                        ((( plane[5]>>i) & 0x1 ) << 5) |
                        ((( plane[6]>>i) & 0x1 ) << 6) |
                        ((( plane[7]>>i) & 0x1 ) << 7) ;
        }

        // this is where the bytewap happens

        *(outpix++) = pix[8];
        *(outpix++) = pix[9];
        *(outpix++) = pix[10];
        *(outpix++) = pix[11];
        *(outpix++) = pix[12];
        *(outpix++) = pix[13];
        *(outpix++) = pix[14];
        *(outpix++) = pix[15];

        *(outpix++) = pix[0];
        *(outpix++) = pix[1];
        *(outpix++) = pix[2];
        *(outpix++) = pix[3];
        *(outpix++) = pix[4];
        *(outpix++) = pix[5];
        *(outpix++) = pix[6];
        *(outpix++) = pix[7];
    }    
}

void p2c_4bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in ) {

    uint8_t pix[16];
    uint16_t *block = (void*)in;
    uint16_t plane[4];

    for( int pixel = 0 ; pixel < pixels_to_convert ; pixel += 16 ) {

//        if( pixel % 320  == 0 )
//            parsecheck();

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

        // this is where the bytewap happens
        *(outpix++) = (pix[9] << 4) | pix[8];
        *(outpix++) = (pix[11] << 4) | pix[10];
        *(outpix++) = (pix[13] << 4) | pix[12];
        *(outpix++) = (pix[15] << 4) | pix[14];

        *(outpix++) = (pix[1] << 4) | pix[0];
        *(outpix++) = (pix[3] << 4) | pix[2];
        *(outpix++) = (pix[5] << 4) | pix[4];
        *(outpix++) = (pix[7] << 4) | pix[6];
    }    
}

void p2c_2bpp( uint8_t *outpix, int pixels_to_convert, uint8_t *in ) {

    uint8_t pix[16];
    uint16_t *block = (void*)in;
    uint16_t plane[2];

    for( int pixel = 0 ; pixel < pixels_to_convert ; pixel += 16 ) {
        
//        if( pixel % 640  == 0 )
//            parsecheck();
        
        plane[0] = *block++;
        plane[1] = *block++;

        // pixel 1 is the sum of the first bit of each of the (4) words raised by two each time

        for( int i = 0 ; i < 16 ; i++ ) {
            pix[15-i] =    ((( plane[0]>>i) & 0x1 ) << 0) | 
                        ((( plane[1]>>i) & 0x1 ) << 1);
        }

        // this is where the bytewap should happen
/*
        *(outpix++) = (pix[12] << 6 ) | ( pix[13] << 4) | (pix[14] << 2) | pix[15];
        *(outpix++) = (pix[8]  << 6 ) | ( pix[9] << 4 ) | (pix[10] << 2) | pix[11];
        *(outpix++) = (pix[4] << 6) | (pix[5] << 4 ) | (pix[6] << 2) | pix[7];
        *(outpix++) = (pix[0] << 6) | (pix[1] << 4 ) | (pix[2] << 2) | pix[3];
        */
        *(outpix++) = (pix[11]  << 6 ) | ( pix[10] << 4 ) | (pix[9] << 2) | pix[8];
        *(outpix++) = (pix[15] << 6 ) | ( pix[14] << 4) | (pix[13] << 2) | pix[12];
        *(outpix++) = (pix[3] << 6) | (pix[2] << 4 ) | (pix[1] << 2) | pix[0];
        *(outpix++) = (pix[7] << 6) | (pix[6] << 4 ) | (pix[5] << 2) | pix[4];
    }    
}




/* VIDEO */

static void vga_scanvideo_switch(struct scanvideo_mode *vga_scanvideo_mode_selected)
{
    /* Bulk of this function comes from Rumbledethumps' Picocomputer 6502 */
    /* https://github.com/picocomputer/rp6502/ */

    /* Warning that there may be memory leak present in scanvideo_setup() */
    /* This could limit the number of resolution changes */

    // Stop and release resources previously held by scanvideo_setup()
    for (int i = 0; i < 3; i++)
    {
        dma_channel_abort(i);
        if (dma_channel_is_claimed(i))
            dma_channel_unclaim(i);
    }
    pio_clear_instruction_memory(pio0);

    // scanvideo_timing_enable is almost able to stop itself
    for (int sm = 0; sm < 4; sm++)
        if (pio_sm_is_claimed(pio0, sm))
            pio_sm_unclaim(pio0, sm);
    scanvideo_timing_enable(false);
    for (int sm = 0; sm < 4; sm++)
        if (pio_sm_is_claimed(pio0, sm))
            pio_sm_unclaim(pio0, sm);

/* Not sure about this so not including for now */
/*
    // begin scanvideo setup with clock setup
    uint32_t clk = vga_scanvideo_mode_selected->default_timing->clock_freq;
    if (clk == 25200000)
        clk = 25200000 * 8; // 201.6 MHz
    else if (clk == 54000000)
        clk = 54000000 * 4; // 216.0 MHz
    else if (clk == 37125000)
        clk = 37125000 * 4; // 148.5 MHz
    assert(clk >= 120000000 && clk <= 266000000);
    if (clk != clock_get_hz(clk_sys))
    {
        main_flush();
        set_sys_clock_khz(clk / 1000, true);
        main_reclock();
    }
*/
    // These two calls are the main scanvideo startup.
    // There's a memory leak in scanvideo_setup which is
    // patched in the fork we use.

    scanvideo_setup(vga_scanvideo_mode_selected);
    scanvideo_timing_enable(true);
}


/** CORE 1 **/

void core1_func() {
    // initialize video and interrupts on core 1
    //scanvideo_setup(&vga_mode_320x240_60);
    //scanvideo_setup(&vga_mode_800x600_54);
    
    current_mode = &vga_mode_640480;
    scanvideo_setup(current_mode);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);

    uint line_num;
    for(;;) {
        do {
            scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
            line_num = scanvideo_scanline_number(scanline_buffer->scanline_id);
            scanline_renderer(scanline_buffer);
            scanvideo_end_scanline_generation(scanline_buffer);
        
        //} while( line_num != 0 );
        //} while( line_num != PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT-1 );
        } while( line_num != ( current_mode->height / current_mode->yscale )-1 );
        _vbls++;
    }

}

void vga_320_4p(scanvideo_scanline_buffer_t *buffer) {

    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    uint linenum_virt = doubleline ? line_num / 2 : line_num;
    short REALX = current_mode->width;
    short REALY = current_mode->height;

    if( Y <  REALY )
        linenum_virt -= doubleline ? ( (REALY/2) - Y) / 2 : ( REALY - Y ) / 2;

    if( linenum_virt < 0 || linenum_virt >= Y ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0x0000;
        *p++ = REALX - 3;
    }
    else {
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixout+(linenum_virt*X/2)); // 4bpp -- two pix per byte, but second half of framebuffer (for background p2c)

        *p++ = COMPOSABLE_RAW_RUN;
        
        colidx = *src++;
        *p++ = palette[(colidx >> 0)&0xf];
        *p++ = REALX - 3;
        *p++ = palette[(colidx >> 0)&0xf];

        *p++ = palette[(colidx >> 4)&0xf];
        *p++ = palette[(colidx >> 4)&0xf];

        *p++ = palette[(colidx >> 8)&0xf];
        *p++ = palette[(colidx >> 8)&0xf];

        *p++ = palette[(colidx >> 12)&0xf];
        *p++ = palette[(colidx >> 12)&0xf];

        *p++ = palette[(colidx >> 16)&0xf];
        *p++ = palette[(colidx >> 16)&0xf];

        *p++ = palette[(colidx >> 20)&0xf];
        *p++ = palette[(colidx >> 20)&0xf];

        *p++ = palette[(colidx >> 24)&0xf];
        *p++ = palette[(colidx >> 24)&0xf];

        *p++ = palette[(colidx >> 28)&0xf];
        *p++ = palette[(colidx >> 28)&0xf];

        for( int i = 8 ; i < X ; i+=8 ) {
            colidx = *src++;            
            *p++ = palette[(colidx >> 0)&0xf];
            *p++ = palette[(colidx >> 0)&0xf];

            *p++ = palette[(colidx >> 4)&0xf];
            *p++ = palette[(colidx >> 4)&0xf];

            *p++ = palette[(colidx >> 8)&0xf];
            *p++ = palette[(colidx >> 8)&0xf];

            *p++ = palette[(colidx >> 12)&0xf];
            *p++ = palette[(colidx >> 12)&0xf];

            *p++ = palette[(colidx >> 16)&0xf];
            *p++ = palette[(colidx >> 16)&0xf];

            *p++ = palette[(colidx >> 20)&0xf];
            *p++ = palette[(colidx >> 20)&0xf];

            *p++ = palette[(colidx >> 24)&0xf];
            *p++ = palette[(colidx >> 24)&0xf];

            *p++ = palette[(colidx >> 28)&0xf];
            *p++ = palette[(colidx >> 28)&0xf];
        }
    }

    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    // end of line with alignment padding
    *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
    *p++ = 0;

    buffer->data_used = ((uint32_t *) p) - buffer->data;
//    buffer->data_used = 650;
    buffer->status = SCANLINE_OK;
}

void vga_320_8p(scanvideo_scanline_buffer_t *buffer) {
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    short REALX = current_mode->width;
    short REALY = current_mode->height;

    uint linenum_virt = doubleline ? line_num / 2 : line_num;

    if( Y <  REALY )
        linenum_virt -= doubleline ? ( (REALY/2) - Y ) / 2 : ( REALY - Y ) / 2;

    if( linenum_virt < 0 || linenum_virt >= Y ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0x0000;
        *p++ = REALX - 3;
    }
    else {
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixout+(linenum_virt*X)); // 8bpp -- one pix per byte

        *p++ = COMPOSABLE_RAW_RUN;
        
        for( int i = 0 ; i < X ; i+=4 ) {
            colidx = *src++;
            *p++ = palette[(colidx >> 0)&0xff];
            if( i == 0 )
                *p++ = REALX - 3;
            *p++ = palette[(colidx >> 0)&0xff];
 
            *p++ = palette[(colidx >> 8)&0xff];
            *p++ = palette[(colidx >> 8)&0xff];
 
            *p++ = palette[(colidx >> 16)&0xff];
            *p++ = palette[(colidx >> 16)&0xff];
 
            *p++ = palette[(colidx >> 24)&0xff];
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

void vga_640_1p(scanvideo_scanline_buffer_t *buffer) {
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    short REALX = current_mode->width;
    short REALY = current_mode->height;

    uint linenum_virt = doubleline ? line_num / 2 : line_num;

    if( Y <  REALY )
        linenum_virt -= doubleline ? ( (REALY/2) - Y ) / 2 : ( REALY - Y ) / 2;

    if( linenum_virt < 0 || linenum_virt >= Y ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0x0000;
        *p++ = X - 3;
    }
    else {
        uint32_t bitmap;
        uint32_t *src = (uint32_t*)(pixout+(linenum_virt*X/8)); // 1bpp -- eight pix per byte

        *p++ = COMPOSABLE_RAW_RUN;

        for( int i = 0 ; i < X ; i += 32 ) {
            bitmap = *src++;

            uint16_t upper = bitmap >> 16;

            *p++ = (bitmap & 128) ? 0 : 0x0fff;
            if( i == 0 )
                *p++ = X-3;
            *p++ = (bitmap & 64) ? 0 : 0x0fff;
            *p++ = (bitmap & 32) ? 0 : 0x0fff;
            *p++ = (bitmap & 16) ? 0 : 0x0fff;
            *p++ = (bitmap & 8) ? 0 : 0x0fff;
            *p++ = (bitmap & 4) ? 0 : 0x0fff;
            *p++ = (bitmap & 2) ? 0 : 0x0fff;
            *p++ = (bitmap & 1) ? 0 : 0x0fff;

            *p++ = (bitmap & 32768) ? 0 : 0x0fff;
            *p++ = (bitmap & 16384) ? 0 : 0x0fff;
            *p++ = (bitmap & 8192) ? 0 : 0x0fff;
            *p++ = (bitmap & 4096) ? 0 : 0x0fff;
            *p++ = (bitmap & 2048) ? 0 : 0x0fff;
            *p++ = (bitmap & 1024) ? 0 : 0x0fff;
            *p++ = (bitmap & 512) ? 0 : 0x0fff;
            *p++ = (bitmap & 256) ? 0 : 0x0fff;

            *p++ = (upper & 128) ? 0 : 0x0fff;
            *p++ = (upper & 64) ? 0 : 0x0fff;
            *p++ = (upper & 32) ? 0 : 0x0fff;
            *p++ = (upper & 16) ? 0 : 0x0fff;
            *p++ = (upper & 8) ? 0 : 0x0fff;
            *p++ = (upper & 4) ? 0 : 0x0fff;
            *p++ = (upper & 2) ? 0 : 0x0fff;
            *p++ = (upper & 1) ? 0 : 0x0fff;

            *p++ = (upper & 32768) ? 0 : 0x0fff;
            *p++ = (upper & 16384) ? 0 : 0x0fff;
            *p++ = (upper & 8192) ? 0 : 0x0fff;
            *p++ = (upper & 4096) ? 0 : 0x0fff;
            *p++ = (upper & 2048) ? 0 : 0x0fff;
            *p++ = (upper & 1024) ? 0 : 0x0fff;
            *p++ = (upper & 512) ? 0 : 0x0fff;
            *p++ = (upper & 256) ? 0 : 0x0fff;

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

void vga_640_2p(scanvideo_scanline_buffer_t *buffer) {
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    short REALX = current_mode->width;
    short REALY = current_mode->height;

    uint linenum_virt = doubleline ? line_num / 2 : line_num;

    if( Y <  REALY )
        linenum_virt -= doubleline ? ( (REALY/2) - Y ) / 2 : ( REALY - Y ) / 2;

    if( linenum_virt < 0 || linenum_virt >= Y ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        //*p++ = doublebuf ? 0x0fff : 0x0000;
        *p++ = 0x0000;
        *p++ = X - 3;
    }
    else {
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixout+(linenum_virt*X/4)); // 2bpp -- four pix per byte

        *p++ = COMPOSABLE_RAW_RUN;

        for( int i = 0 ; i < X ; i += 16 ) {
            colidx = *src++;            
            *p++ = palette[(colidx >> 0)&0x3];
            if( i == 0 )
                *p++ = X-3;
            *p++ = palette[(colidx >> 2)&0x3];
            *p++ = palette[(colidx >> 4)&0x3];
            *p++ = palette[(colidx >> 6)&0x3];

            *p++ = palette[(colidx >> 8)&0x3];
            *p++ = palette[(colidx >> 10)&0x3];
            *p++ = palette[(colidx >> 12)&0x3];
            *p++ = palette[(colidx >> 14)&0x3];

            *p++ = palette[(colidx >> 16)&0x3];
            *p++ = palette[(colidx >> 18)&0x3];
            *p++ = palette[(colidx >> 20)&0x3];
            *p++ = palette[(colidx >> 22)&0x3];

            *p++ = palette[(colidx >> 24)&0x3];
            *p++ = palette[(colidx >> 26)&0x3];
            *p++ = palette[(colidx >> 28)&0x3];
            *p++ = palette[(colidx >> 30)&0x3];
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

void vga_640_4p(scanvideo_scanline_buffer_t *buffer) {
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint16_t *p = (uint16_t *) buffer->data;

    short REALX = current_mode->width;
    short REALY = current_mode->height;

    uint linenum_virt = doubleline ? line_num / 2 : line_num;

    if( Y <  REALY )
        linenum_virt -= doubleline ? ( (REALY/2) - Y ) / 2 : ( REALY - Y ) / 2;

    if( linenum_virt < 0 || linenum_virt >= Y ) { // blank
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = 0x0000;
        *p++ = REALX - 3;
    }
    else {
        uint32_t colidx;
        uint32_t *src = (uint32_t*)(pixout+(linenum_virt*X/2)); // 4bpp -- two pix per byte

        *p++ = COMPOSABLE_RAW_RUN;
        
        colidx = *src++;
        *p++ = palette[(colidx >> 0)&0xf];
        *p++ = REALX - 3;

        *p++ = palette[(colidx >> 4)&0xf];

        *p++ = palette[(colidx >> 8)&0xf];

        *p++ = palette[(colidx >> 12)&0xf];

        *p++ = palette[(colidx >> 16)&0xf];

        *p++ = palette[(colidx >> 20)&0xf];

        *p++ = palette[(colidx >> 24)&0xf];

        *p++ = palette[(colidx >> 28)&0xf];

        for( int i = 8 ; i < X ; i+=8 ) {
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


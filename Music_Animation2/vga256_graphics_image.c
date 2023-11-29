/**
 * Hunter Adams (vha3@cornell.edu)
 * converted to 320x240 with 256 colors by Bruce; brue.land@cornell.edu
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 * 
 *  - GPIO 08 ---> 330 ohm resistor ---> VGA Blue lo-bit |__ both wired to 150 ohm to ground 
 *  - GPIO 09 ---> 220 ohm resistor ---> VGA Blue hi_bit |   and to VGA blue
 * 
 *  - GPIO 10 ---> 1000 ohm resistor ---> VGA Green lo-bit |__ three wired to 100 ohm to ground
 *  - GPIO 11 ---> 680 ohm resistor ---> VGA Green mid_bit |   and to VGA Green
 *  - GPIO 12 ---> 330 ohm resistor ---> VGA Green hi_bit  |   
 * 
 *  - GPIO 13 ---> 1000 ohm resistor ---> VGA Red lo-bit |__ three wired to 100 ohm to ground
 *  - GPIO 14 ---> 680 ohm resistor ---> VGA Red mid_bit |   and to VGA red
 *  - GPIO 15 ---> 330 ohm resistor ---> VGA Red hi_bit  |   
 * 
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0 to 3 on PIO instance 0
 *  - DMA channels 0, 1, 2, 3 data send to two PIO
 *  - 76.8 kBytes of RAM (for pixel color data)
 * color encoding: bits 7:5 red; 4:2 green; 1:0 blue
 *
 * Protothreads v1.1.1
 * NOTE: Serial TX gpio0 must be looped back to serial RX gpio1
 * CORE 0
 * graphics demo thread -- draw an image from serial input
 * the usual blinky thread for a heartbeat
 * Core 1
 * image send thread -- builds a simple image and sends via serial
 * 
 */
// ==========================================
// === VGA graphics library
// ==========================================
#include "vga256_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

// ==========================================
// === protothreads globals
// ==========================================
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "string.h"
// protothreads header
#include "pt_cornell_rp2040_v1_1_1.h"
// 
// serial thread signals image thread to send
int start_image ;
// the image draw array [y][x]
// all normal pixels draw commands are in color range 0x00-0xff
// frame start command is 0x100
// line start command is 0x200
// frame end commnad is 0x300 
#define image_x_size (50)
#define image_y_size (75)
short image_array[image_y_size][image_x_size] ;
#define frame_start (0x100)
#define line_start  (0x200)
#define frame_end   (0x300)
// the upper-left corner of the image
// low two hex digits give postion
#define set_x       (0x400)
#define set_y       (0x800)
// upper-left corner of image
short image_x_offset = 50 ;
short image_y_offset = 15 ;

// current command being sent
short draw_cmd ;
// serial send macro
//#define send_cmd do{printf("%d\n\r", draw_cmd)}while(0)


// ==================================================
// === graphics demo -- RUNNING on core 0
// ==================================================
static PT_THREAD (protothread_graphics(struct pt *pt)) {
    PT_BEGIN(pt);

    // background
    fillRect(0, 0, 319, 239, BLACK); // 

    // Draw some filled rectangles
    fillRect(0, 0, 76, 10, BLUE); // blue box
    fillRect(100, 0, 150, 10, WHITE); // red box
    //fillRect(200, 0, 76, 10, GREEN); // green box

    // Write some text
    setTextColor(WHITE) ;
    setCursor(10, 1) ;
    setTextSize(1) ;
    writeString("ECE 4760") ;

    setTextColor(BLACK) ;
    setCursor(102, 1) ;
    setTextSize(1) ;
    writeString("VGA 320x240 8-bit color ") ;

    static int cmd ;
    static short  draw_x, draw_y ;
    draw_x = image_x_offset ;
    draw_y = image_y_offset ;
    while(true) {
        // get draw command from buffer
        // frame_start is non-blocking so that other threads run
        serial_read ;
        sscanf(pt_serial_in_buffer, " %x ",  &cmd) ;
        if (cmd == frame_start){
            draw_x = image_x_offset ;
            draw_y = image_y_offset-1 ;
            while (cmd != frame_end){
                // this serial read is blocking for speed
                scanf("%x", &cmd) ;
                
                if((cmd & set_x) == set_x){
                  image_x_offset = cmd & 0xff ;
                  draw_x = image_x_offset ;
                }
                else if((cmd & set_y) == set_y){
                  image_y_offset = cmd & 0xff ;
                  draw_y = image_y_offset - 1;
                }
                
                else if (cmd == line_start){
                    draw_x = image_x_offset;
                    draw_y = draw_y + 1 ;
                }
                else if (cmd < 0x100){
                    drawPixel(draw_x, draw_y, cmd) ;
                    draw_x = draw_x + 1 ;
                }
            } // end while !frame_end

        } // end if frame start       
   } // end while(true)
   PT_END(pt);
} // image draw thread

// ==================================================
// === toggle25 thread on core 0
// ==================================================
// the on-board LED blinks
static PT_THREAD (protothread_toggle25(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool LED_state = false ;
    
     // set up LED p25 to blink
     gpio_init(25) ;	
     gpio_set_dir(25, GPIO_OUT) ;
     gpio_put(25, true);
     // data structure for interval timer
     PT_INTERVAL_INIT() ;

      while(1) {
        // yield time 0.1 second
        //PT_YIELD_usec(100000) ;
        PT_YIELD_INTERVAL(100000) ;

        // toggle the LED on PICO
        LED_state = LED_state? false : true ;
        gpio_put(25, LED_state);

        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // blink thread


// ==================================================
// === user's serial input thread on core 0
// ==================================================
// serial_read an serial_write do not block any thread
// except this one

static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
      
      //
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input 1 to start image send: ");
        // spawn a thread to do the non-blocking write
        //serial_write ;

        // spawn a thread to do the non-blocking serial read
         serial_read ;
        sscanf(pt_serial_in_buffer, " %d ",  &start_image) ;
        
    } // END WHILE(1)
  PT_END(pt);
} // serial thread

// ==================================================
// === image generate/send thread on core 1
// ==================================================
// build an image and send it to core 0

static PT_THREAD (protothread_send_image(struct pt *pt))
{
    PT_BEGIN(pt);
      //
      for(int y=0; y<image_y_size; y++){
            for(int x=0; x<image_x_size; x++){
                if(x==0 | y==0 | x==image_x_size-1 | y==image_y_size-1) image_array[y][x] = WHITE ;
                if(x>=10 && x<= 20 && y>=10 && y<= 20) image_array[y][x] = RED ;
                if(x>=22 && x<= 30 && y>=30 && y<= 60) image_array[y][x] = rgb(7,7,2) ;
                if(x>=35 && x<= 40 && y>=50 && y<= 65) image_array[y][x] = rgb(4,2,1) ;
            }
      }

      start_image = 1; 

      PT_YIELD_usec(100000) ;

      while(1) {
        // check start flag
        if(start_image){
            // send image -- by setting draw_cmd=xfxxx, then waiting 10 uSec
            draw_cmd = frame_start ;
            //send_cmd ;
            printf("%d\n\r", draw_cmd) ;
            PT_YIELD_usec(10000) ;
            // now the image
            for(int y=0; y<image_y_size; y++){
                for(int x=0; x<image_x_size; x++){
                    //
                    if(x==0) {
                        //PT_YIELD_usec(10) ;
                        draw_cmd = line_start ;
                        printf("%d\n\r", draw_cmd) ;
                    }
                    //PT_YIELD_usec(10) ;
                    draw_cmd = image_array[y][x];
                    printf("%d\n\r", draw_cmd) ;
                }
            }
            // end frame
            //PT_YIELD_usec(10) ;
            draw_cmd = frame_end ;
            printf("%d\n\r", draw_cmd) ;
            // clear start flag
            start_image = 0 ;
        }
        else{
            PT_YIELD(pt) ;
        }           
    } // END WHILE(1)
  PT_END(pt);
} // serial thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){ 
  //
  //  === add threads  ====================
  // for core 1
  pt_add_thread(protothread_send_image) ;
  //
  // === initalize the scheduler ==========
  pt_schedule_start ;
  // NEVER exits
  // ======================================
}

// ========================================
// === core 0 main
// ========================================
int main(){
  // set the clock
  //set_sys_clock_khz(250000, true); // 171us
  // start the serial i/o
  stdio_init_all() ;
  // announce the threader version on system reset
  //printf("\n\rProtothreads RP2040 v1.11 two-core\n\r");

  // Initialize the VGA screen
  initVGA() ;
     
  // start core 1 threads
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // === config threads ========================
  // for core 0
  pt_add_thread(protothread_graphics);
  pt_add_thread(protothread_toggle25);
  //pt_add_thread(protothread_serial) ;
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;
  // NEVER exits
  // ===========================================
} // end main
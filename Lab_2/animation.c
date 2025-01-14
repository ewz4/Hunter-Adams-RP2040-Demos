
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

// LED 
#define LED 25

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;
fix15 boid0_turnfactor;
fix15 boid0_visualRange;
fix15 boid0_protectedRange;
fix15 boid0_centeringfactor;
fix15 boid0_avoidfactor;
fix15 boid0_matchingfactor;
fix15 boid0_maxspeed;
fix15 boid0_minspeed;
fix15 speed;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, fix15* turnfactor, fix15* maxspeed, fix15* minspeed, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
  // Choose left or right
  if (direction) *vx = int2fix15(3) ;
  else *vx = int2fix15(-3) ;
  // Moving down
  *vy = int2fix15(-3) ;
  *turnfactor = float2fix15(0.2);
  *maxspeed = int2fix15(6);
  *minspeed = int2fix15(3);
}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy, fix15* turnfactor, fix15* maxspeed, fix15* minspeed)
{
  // Reverse direction if we've hit a wall
  // if (hitTop(*y)) {
  //   *vy = (-*vy) ;
  //   *y  = (*y + int2fix15(5)) ;
  // }
  // if (hitBottom(*y)) {
  //   *vy = (-*vy) ;
  //   *y  = (*y - int2fix15(5)) ;
  // } 
  // if (hitRight(*x)) {
  //   *vx = (-*vx) ;
  //   *x  = (*x - int2fix15(5)) ;
  // }
  // if (hitLeft(*x)) {
  //   *vx = (-*vx) ;
  //   *x  = (*x + int2fix15(5)) ;
  // } 

  if (*x < int2fix15(100)) {
    *vx = *vx + *turnfactor;
  }
  if (*x > int2fix15(540)) {
    *vx = *vx - *turnfactor;
  }
  if (*y < int2fix15(100)) {
    *vy = *vy + *turnfactor;
  }
  if (*y > int2fix15(380)) {
    *vy = *vy - *turnfactor;
  }

  speed = sqrtfix(multfix15(*vx, *vx) + multfix15(*vy,*vy));
  if (speed > *maxspeed) {
    *vx = multfix15(divfix(*vx, speed), *maxspeed);
    *vy = multfix15(divfix(*vy, speed), *maxspeed);
  }
  if (speed < *minspeed) {
    *vx = multfix15(divfix(*vx, speed), *minspeed);
    *vy = multfix15(divfix(*vy, speed), *minspeed);
  }

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;

  // printf("x: %d\n", fix2int15(*x));
  // printf("y: %d\n", fix2int15(*y));
  // printf("vx: %d\n", fix2int15(*vx));
  // printf("vy: %d\n", fix2int15(*vy));
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    gpio_put(LED, !gpio_get(LED));

    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-7: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 8)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;
    static int total_time = 0;
    static int counter = 0;
    static int N_boids = 1;
    static int frame_rate_text = FRAME_RATE;
    char str1[10000];
    char str2[10000];
    char str3[10000];
    char str4[10000];

    // Spawn a boid
    spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, &boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed, 0);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase boid
      drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, BLACK);
      // update boid's position and velocity
      wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, &boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed) ;
      // draw the boid at its new position
      drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, color); 
      // draw the boundaries
      drawArena() ;

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

      //Display text: Number of boids, frame rate, time elapsed
      
      total_time = time_us_32() / 1000000;
      printf("%d\n", total_time);
      printf("%s\n", str1);
      sprintf(str1, "Time Elapsed = %d seconds", total_time);

      printf("%d\n", spare_time);
      printf("%s\n", str2);
      sprintf(str2, "Spare Time = %d us", spare_time);

      printf("%d\n", frame_rate_text);
      printf("%s\n", str3);
      sprintf(str3, "Frame Rate = %d us/frame", frame_rate_text);

      printf("%d\n", N_boids);
      printf("%s\n", str4);
      sprintf(str4, "Number of boids = %d", N_boids);



 
      if (counter > 30) {
        fillRect(0, 0, 600, 99, BLACK);
        setCursor(50,50);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str1);

        setCursor(300,50);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str2);

        setCursor(50,75);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str3);

        setCursor(300,75);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str4);

        counter = 0;
      }

      counter++;

      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
      // PT_YIELD_usec(1000000);
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// // Animation on core 1
// static PT_THREAD (protothread_anim1(struct pt *pt))
// {
//     // Mark beginning of thread
//     PT_BEGIN(pt);

//     // Variables for maintaining frame rate
//     static int begin_time ;
//     static int spare_time ;

//     // Spawn a boid
//     spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

//     while(1) {
//       // Measure time at start of thread
//       begin_time = time_us_32() ;      
//       // erase boid
//       drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, BLACK);
//       // update boid's position and velocity
//       wallsAndEdges(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy) ;
//       // draw the boid at its new position
//       drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, color); 
//       // delay in accordance with frame rate
//       spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
//       // yield for necessary amount of time
//       PT_YIELD_usec(spare_time) ;
//      // NEVER exit while
//     } // END WHILE(1)
//   PT_END(pt);
// } // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
// void core1_main(){
//   // Add animation thread
//   pt_add_thread(protothread_anim1);
//   // Start the scheduler
//   pt_schedule_start ;

// }

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // Map LED to GPIO port, make it low
  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 0);

  // start core 1 
  // multicore_reset_core1();
  // multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 

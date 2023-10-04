
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

// Build boid struct
struct boid {
    fix15 x ;
    fix15 y ;
    fix15 vx ;
    fix15 vy ;
};

#define N_boids 600

struct boid boids[N_boids];
int curr_N_boids = 0;
int i = 0;


struct predator {
    fix15 x ;
    fix15 y ;
    fix15 vx ;
    fix15 vy ;
};

#define N_predators 10
int curr_N_predators = 0;
struct predator predators[N_predators];
int k = 0;
int l = 0;
fix15 predatory_range = int2fix15(100);
fix15 squared_predator_distance ; 
fix15 predatory_range_square = int2fix15(10000);
fix15 predator_dx ;
fix15 predator_dy ;
int num_predators = 0;
fix15 predator_turnfactor = float2fix15(0.5);

// fix15 distance_between_boids[45]; //45 combinations of boids

// accumulator vairables
fix15 xpos_avg ;
fix15 ypos_avg ;
fix15 xvel_avg ;
fix15 yvel_avg ;
int neighboring_boids ;
fix15 close_dx ;
fix15 close_dy ;

// Boid on core 0
fix15 boid0_turnfactor = float2fix15(0.2);
fix15 boid0_visualRange = int2fix15(40);
fix15 boid0_protectedRange = int2fix15(8);
fix15 boid0_centeringfactor = float2fix15(0.0005);
fix15 boid0_avoidfactor = float2fix15(0.05);
fix15 boid0_matchingfactor = float2fix15(0.05);
fix15 boid0_maxspeed = int2fix15(6);
fix15 boid0_minspeed = int2fix15(3);
fix15 speed;
fix15 boid0_protectedRangeSquared = int2fix15(64);
fix15 boid0_visualRangeSquared = int2fix15(1600);
fix15 dx;
fix15 dy;
fix15 squared_distance;
fix15 neighboring_boids_div;


// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;

// Margin Size
int x_margin_left_box = 100;
int x_margin_right_box = 540;
int x_change_margin_box = 440; 
int y_margin_top_box = 100;
int y_margin_bottom_box = 380;
int y_change_margin_box = 280;
int should_draw = 0; //Margins default to zero

// Vertical Line Size
int x_margin_left_V_line = 200;
int x_margin_right_V_line = 440;
int y_margin_top_line = 0;
int y_change_margin_line = 480;


// Screen Size
int y_screen_top = 0;
int y_screen_bottom = 480;
int x_screen_left = 0;
int x_screen_right = 640;



// Create all boids
void spawnBoids(fix15* turnfactor, fix15* maxspeed, fix15* minspeed, int direction)
{
  // Start in center of screen
  for (i = 0; i < curr_N_boids; i++)
  {
    boids[i].x = int2fix15(rand() % 640);
    boids[i].y = int2fix15(rand() % 480);
    boids[i].vx = int2fix15(rand() % 3 + 3);
    boids[i].vy = int2fix15(rand() % 3 + 3);
  }
  for (k = 0; k < curr_N_predators; k++)
  {
    predators[k].x = int2fix15(rand() % 640);
    predators[k].y = int2fix15(rand() % 480);
    predators[k].vx = int2fix15(rand() % 3 + 3);
    predators[k].vy = int2fix15(rand() % 3 + 3);
  }
  k = 0;
}

// Draw the boundaries
void drawArena(int should_draw) 
{
  if (should_draw == 1)
  {
    // Clear vertical lines
    drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
    drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;

    drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, WHITE) ;
    drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, WHITE) ;
    drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, WHITE) ;
    drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, WHITE) ;
  }
  else if (should_draw == 2)
  {
    drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, WHITE) ;
    drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, WHITE) ;

  }
  else if (should_draw == 0)
  {
    
  }
}

// Detect wallstrikes, update velocity and position
void boid_algo(fix15* turnfactor, fix15* maxspeed, fix15* minspeed, int i)
{
  // For every other boid in the flock . . .
  for (int j = 0; j < curr_N_boids; j++) 
  {
    // For each boid other than the current boid
    if (j != i) 
    {
      // Compute differences in x and y coordinates
      dx = boids[i].x - boids[j].x;
      dy = boids[i].y - boids[j].y;
      // Are both those differences less than the visual range?
      if (absfix15(dx) < boid0_visualRange && absfix15(dy) < boid0_visualRange)
      {
        // If so, calculate the squared distance
        squared_distance = multfix15(dx,dx) + multfix15(dy,dy);

        // Is squared distance less than the protected range?
        if (squared_distance < boid0_protectedRangeSquared)
        {
            // If so, calculate difference in x/y-coordinates to nearfield boid
            close_dx += boids[i].x - boids[j].x;
            close_dy += boids[i].y - boids[j].y;
        }
        // If not in protected range, is the boid in the visual range?
        else if (squared_distance < boid0_visualRangeSquared)
        {
            // Add other boid's x/y-coord and x/y vel to accumulator variables
            xpos_avg += boids[j].x;
            ypos_avg += boids[j].y;
            xvel_avg += boids[j].vx;
            yvel_avg += boids[j].vy;
            
            // Increment number of boids within visual range
            neighboring_boids++;
        }
      }
    }
  }

  // If there were any boids in the visual range
  if (neighboring_boids > 0)
  {
    // Divide accumulator variables by number of boids in visual range
    neighboring_boids_div = divfix(int2fix15(1), int2fix15(neighboring_boids));
    xpos_avg = multfix15(xpos_avg,neighboring_boids_div);
    ypos_avg = multfix15(ypos_avg,neighboring_boids_div);
    xvel_avg = multfix15(xvel_avg,neighboring_boids_div);
    yvel_avg = multfix15(yvel_avg,neighboring_boids_div);

    // Add the centering/matching contributions to velocity
    boids[i].vx = (boids[i].vx + 
                  multfix15(xpos_avg - boids[i].x, boid0_centeringfactor) + 
                  multfix15(xvel_avg - boids[i].vx, boid0_matchingfactor));
    boids[i].vy = (boids[i].vy + 
                  multfix15(ypos_avg - boids[i].y, boid0_centeringfactor) + 
                  multfix15(yvel_avg - boids[i].vy, boid0_matchingfactor));
  }

  // Add the avoidance contribution to velocity
  boids[i].vx = boids[i].vx + multfix15(close_dx, boid0_avoidfactor);
  boids[i].vy = boids[i].vy + multfix15(close_dy, boid0_avoidfactor);
  
  // If the boid is near an edge, make it turn by turnfactor
  // (this describes a box, will vary based on boundary conditions)
  if (should_draw == 0) //wrap everywhere
  {
    if (boids[i].y < int2fix15(y_screen_top))
    {
        boids[i].y = int2fix15(y_screen_bottom);
    }
    if (boids[i].y > int2fix15(y_screen_bottom))
    {
        boids[i].y = int2fix15(y_screen_top);
    } 
    if (boids[i].x < int2fix15(x_screen_left))
    {
        boids[i].x = int2fix15(x_screen_right);
    }
    if (boids[i].x > int2fix15(x_screen_right))
    {
        boids[i].x = int2fix15(x_screen_left);
    }  
  }
  else if (should_draw == 1) // if should_draw == 1 --> box
  {
    if (boids[i].y < int2fix15(y_margin_top_box))
    {
      boids[i].vy = boids[i].vy + *turnfactor;
    }
    if (boids[i].y > int2fix15(y_margin_bottom_box))
    {
      boids[i].vy = boids[i].vy - *turnfactor;
    } 
    if (boids[i].x < int2fix15(x_margin_left_box))
    {
      boids[i].vx = boids[i].vx + *turnfactor;
    }
    if (boids[i].x > int2fix15(x_margin_right_box))
    {
      boids[i].vx = boids[i].vx - *turnfactor;
    }  
  }
  else // should_draw == 2 --> draw 2 lines
  {
    if (boids[i].y < int2fix15(y_screen_top))
    {
        boids[i].y = int2fix15(y_screen_bottom);
    }
    if (boids[i].y > int2fix15(y_screen_bottom))
    {
        boids[i].y = int2fix15(y_screen_top);
    } 
    if (boids[i].x < int2fix15(x_margin_left_V_line))
    {
      boids[i].vx = boids[i].vx + *turnfactor;
    }
    if (boids[i].x > int2fix15(x_margin_right_V_line))
    {
      boids[i].vx = boids[i].vx - *turnfactor;
    }   
  }
 
  
  //////////////////////////////////
  // Predator Part
    
  for (k = 0; k < curr_N_predators; k++)
  {
    
    // Compute the differences in x and y coordinates
    dx = boids[i].x - predators[k].x;
    dy = boids[i].y - predators[k].y;

    // Are both those differences less than the predatory range?
    if (absfix15(dx) < predatory_range && absfix15(dy) < predatory_range)
    {
      // If so, calculate the squared distance to the predator
      squared_predator_distance = multfix15(dx,dx) + multfix15(dy,dy);

      // Is the squared distance less than the predatory range squared?
      if (squared_predator_distance < predatory_range_square)
      {
        predator_dx += boids[i].x - predators[k].x;
        predator_dy += boids[i].y - predators[k].y;

        // Increment the number of predators in the boid's predatory range
        num_predators += 1;
      }
    }
  }

  // If there were any predators in the predatory range, turn away
  if (num_predators > 0)
  {
    if (predator_dy > 0)
    {
      boids[i].vy = boids[i].vy + predator_turnfactor;
    }
    if (predator_dy < 0)
    {
      boids[i].vy = boids[i].vy - predator_turnfactor;
    }
    if (predator_dx > 0)
    {
      boids[i].vx = boids[i].vx + predator_turnfactor;
    }
    if (predator_dx < 0)
    {
      boids[i].vx = boids[i].vx - predator_turnfactor;
    }
  }
  //////////////////////////////////


  //Calculate the boid's speed
  //Slow step! Lookup the "alpha max plus beta min" algorithm  
  speed = sqrtfix(multfix15(boids[i].vx,boids[i].vx) + 
                  multfix15(boids[i].vy,boids[i].vy));

  if (speed > *maxspeed) {
    boids[i].vx = multfix15(divfix(boids[i].vx, speed), *maxspeed);
    boids[i].vy = multfix15(divfix(boids[i].vy, speed), *maxspeed);
  }
  if (speed < *minspeed) {
    boids[i].vx = multfix15(divfix(boids[i].vx, speed), *minspeed);
    boids[i].vy = multfix15(divfix(boids[i].vy, speed), *minspeed);
  }

  // Update position using velocity
  boids[i].x = boids[i].x + boids[i].vx;
  boids[i].y = boids[i].y + boids[i].vy;


//   printf("x: %d\n", fix2int15(boids[i].x));
//   printf("y: %d\n", fix2int15(boids[i].y));
//   printf("vx: %d\n", fix2int15(boids[i].vx));
//   printf("vy: %d\n", fix2int15(boids[i].vy));
}

void predator_algo(fix15* turnfactor, fix15* maxspeed, fix15* minspeed, int l)
{
  // If the boid is near an edge, make it turn by turnfactor
  // (this describes a box, will vary based on boundary conditions)
  if (should_draw == 0) //wrap everywhere
  {
    if (predators[l].y < int2fix15(y_screen_top))
    {
        predators[l].y = int2fix15(y_screen_bottom);
    }
    if (predators[l].y > int2fix15(y_screen_bottom))
    {
        predators[l].y = int2fix15(y_screen_top);
    } 
    if (predators[l].x < int2fix15(x_screen_left))
    {
        predators[l].x = int2fix15(x_screen_right);
    }
    if (predators[l].x > int2fix15(x_screen_right))
    {
        predators[l].x = int2fix15(x_screen_left);
    }  
  }
  else if (should_draw == 1) // if should_draw == 1 --> box
  {
    if (predators[l].y < int2fix15(y_margin_top_box))
    {
      predators[l].vy = predators[l].vy + *turnfactor;
    }
    if (predators[l].y > int2fix15(y_margin_bottom_box))
    {
      predators[l].vy = predators[l].vy - *turnfactor;
    } 
    if (predators[l].x < int2fix15(x_margin_left_box))
    {
      predators[l].vx = predators[l].vx + *turnfactor;
    }
    if (predators[l].x > int2fix15(x_margin_right_box))
    {
      predators[l].vx = predators[l].vx - *turnfactor;
    }  
  }
  else // should_draw == 2 --> draw 2 lines
  {
    if (predators[l].y < int2fix15(y_screen_top))
    {
        predators[l].y = int2fix15(y_screen_bottom);
    }
    if (predators[l].y > int2fix15(y_screen_bottom))
    {
        predators[l].y = int2fix15(y_screen_top);
    } 
    if (predators[l].x < int2fix15(x_margin_left_V_line))
    {
      predators[l].vx = predators[l].vx + *turnfactor;
    }
    if (predators[l].x > int2fix15(x_margin_right_V_line))
    {
      predators[l].vx = predators[l].vx - *turnfactor;
    }   
  }
  //////////////////////////////////

  //Calculate the boid's speed
  //Slow step! Lookup the "alpha max plus beta min" algorithm  
  speed = sqrtfix(multfix15(predators[l].vx,predators[l].vx) + 
                  multfix15(predators[l].vy,predators[l].vy));

  if (speed > *maxspeed) {
    predators[l].vx = multfix15(divfix(predators[l].vx, speed), *maxspeed);
    predators[l].vy = multfix15(divfix(predators[l].vy, speed), *maxspeed);
  }
  if (speed < *minspeed) {
    predators[l].vx = multfix15(divfix(predators[l].vx, speed), *minspeed);
    predators[l].vy = multfix15(divfix(predators[l].vy, speed), *minspeed);
  }

  // Update position using velocity
  predators[l].x = predators[l].x + predators[l].vx;
  predators[l].y = predators[l].y + predators[l].vy;


  //   printf("x: %d\n", fix2int15(boids[i].x));
  //   printf("y: %d\n", fix2int15(boids[i].y));
  //   printf("vx: %d\n", fix2int15(boids[i].vx));
  //   printf("vy: %d\n", fix2int15(boids[i].vy));
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    gpio_put(LED, !gpio_get(LED));
    PT_BEGIN(pt);
    // stores user input
    static int int_input ;
    static float float_input;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;

    static char cmd[16], arg1[16];
    static char* token ;
      while(1) {
        // // print prompt
        // sprintf(pt_serial_out_buffer, "input a number in the range 1-7: ");
        // // non-blocking write
        // serial_write ;
        // // spawn a thread to do the non-blocking serial read
        // serial_read ;
        // // convert input string to number
        // sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // // update boid color
        // if ((user_input > 0) && (user_input < 8)) {
        //   color = (char)user_input ;
        // }
        // print prompt
        // sprintf(pt_serial_out_buffer, "Vertical lines = 2, Margins = 1, Off = 0: ");
        // // non-blocking write
        // serial_write ;
        // // spawn a thread to do the non-blocking serial read
        // serial_read ;
        // // convert input string to number
        // sscanf(pt_serial_in_buffer,"%d", &int_input) ;
        // // update boid color
        // if (int_input == 2) 
        // {
        //     should_draw = 2;
        //     drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, BLACK) ;
        //     drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, BLACK) ;
        //     drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, BLACK) ;
        //     drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, BLACK) ;
        // }
        // else if (int_input == 1)
        // {  
        //     should_draw = 1;
        //     drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
        //     drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
        // }
        // else if (int_input == 0)
        // {
        //     should_draw = 0;
        //     drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, BLACK) ;
        //     drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, BLACK) ;
        //     drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, BLACK) ;
        //     drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, BLACK) ;

        //     drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
        //     drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
        // }

        // sprintf(pt_serial_out_buffer, "Enter turn factor: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%f", &float_input) ;
        // boid0_turnfactor = float2fix15(float_input);

        // sprintf(pt_serial_out_buffer, "Enter visual range: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%d", &int_input) ;
        // boid0_visualRange = int2fix15(int_input);
        // boid0_visualRangeSquared = int2fix15(int_input * int_input);

        // sprintf(pt_serial_out_buffer, "Enter protected range: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%d", &int_input) ;
        // boid0_protectedRange = int2fix15(int_input);
        // boid0_protectedRangeSquared = int2fix15(int_input * int_input);

        // sprintf(pt_serial_out_buffer, "Enter centering factor: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%f", &float_input) ;
        // boid0_centeringfactor = float2fix15(float_input);

        // sprintf(pt_serial_out_buffer, "Enter avoid factor: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%f", &float_input) ;
        // boid0_avoidfactor = float2fix15(float_input);

        // sprintf(pt_serial_out_buffer, "Enter matching factor: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%f", &float_input) ;
        // boid0_matchingfactor = float2fix15(float_input);

        // sprintf(pt_serial_out_buffer, "Enter max speed: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%d", &int_input) ;
        // boid0_maxspeed = int2fix15(int_input);

        // sprintf(pt_serial_out_buffer, "Enter min speed: ");
        // serial_write ;
        // serial_read ;
        // sscanf(pt_serial_in_buffer,"%d", &int_input) ;
        // boid0_minspeed = int2fix15(int_input);

        // sprintf(pt_serial_out_buffer, "m");
        // // non-blocking write
        // serial_write ;
        // // spawn a thread to do the non-blocking serial read
        // serial_read ;
        // // convert input string to number
        // sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // // update boid color
        // if (user_input == 1) 
        // {
        //     should_draw = true;
        // }
        // else if (user_input == 0)
        // {  
        //     should_draw = false;
        // }
        // print prompt
        sprintf(pt_serial_out_buffer, "Enter Command> ");
        // spawn a thread to do the non-blocking write
        serial_write ;

        // spawn a thread to do the non-blocking serial read
        serial_read ;
        //sscanf(pt_serial_in_buffer, "%s %f %f %f %f", arg0, &arg1, arg2, arg3, arg4) ;
        // tokenize
        token = strtok(pt_serial_in_buffer, " ");
        strcpy(cmd, token) ;
        token = strtok(NULL, " ");
        strcpy(arg1, token) ;


        // parse by command
        if(strcmp(cmd,"help")==0){
            // List commands
            printf("draw line\n\r") ;
            printf("draw box\n\r") ;
            printf("draw none\n\r") ;
            printf("turnfactor\n\r") ;
            printf("visualrange\n\r") ;
            printf("protectedrange\n\r") ;
            printf("centeringfactor\n\r") ;
            printf("avoidfactor\n\r") ;
            printf("matchingfactor\n\r") ;
            printf("numberBoids\n\r") ;
            printf("numberPredators\n\r") ;
        }
        else if(strcmp(cmd,"draw")==0){
            if(strcmp(arg1,"line")==0){
                should_draw = 2;
                drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, BLACK) ;
                drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, BLACK) ;
                drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, BLACK) ;
                drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, BLACK) ;
            } else if (strcmp(arg1,"box")==0){  
                should_draw = 1;
                drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
                drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
            } else if (strcmp(arg1,"none")==0)
            {
                should_draw = 0;
                drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, BLACK) ;
                drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, BLACK) ;
                drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, BLACK) ;
                drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, BLACK) ;

                drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
                drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK) ;
            }
        }
        //
        else if(strcmp(cmd,"turnfactor")==0){
            if(arg1 != NULL){
                boid0_turnfactor = float2fix15(atof(arg1));
            }
        }
        else if(strcmp(cmd,"visualrange")==0){
            if(arg1 != NULL){
                boid0_visualRange = int2fix15(atoi(arg1));
            }
        }
        else if(strcmp(cmd,"protectedrange")==0){
            if(arg1 != NULL){
                boid0_protectedRange = int2fix15(atoi(arg1));
            }
        }
        else if(strcmp(cmd,"centeringfactor")==0){
            if(arg1 != NULL){
                boid0_centeringfactor = float2fix15(atof(arg1));
            }
        }
        else if(strcmp(cmd,"avoidfactor")==0){
            if(arg1 != NULL){
                boid0_avoidfactor = float2fix15(atof(arg1));
            }
        }
        else if(strcmp(cmd,"matchingfactor")==0){
            if(arg1 != NULL){
                boid0_matchingfactor = float2fix15(atof(arg1));
            }
        }
        else if(strcmp(cmd,"numberBoids")==0){
            if(arg1 != NULL)
            {
              for (int i = 0; i < curr_N_boids; i++)
              {
                drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, BLACK);
              }

              for (l = 0; l < curr_N_predators; l++)
              {
                // erase boid
                drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
              }
              curr_N_boids = (int)(atof(arg1));
              spawnBoids(&boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed, 0);
            }
        }
        else if(strcmp(cmd,"numberPredators")==0){
            if(arg1 != NULL){
              for (int i = 0; i < curr_N_boids; i++)
              {
                drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, BLACK);
              }

              for (l = 0; l < curr_N_predators; l++)
              {
                // erase boid
                drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
              }
              
              curr_N_predators = (int)(atof(arg1));
              spawnBoids(&boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed, 0);
            }
        }
        else printf("Huh?\n\r") ;
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
    char str1[50];
    char str2[50];
    char str3[50];
    char str4[50];

    curr_N_boids = 100;
    curr_N_predators = 2;

    // Spawn a boid
    spawnBoids(&boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed, 0);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32();  
      for (int i = 0; i < curr_N_boids; i++)
      {
        // Zero all accumulator variables
        xpos_avg = int2fix15(0);
        ypos_avg = int2fix15(0);
        xvel_avg = int2fix15(0);
        yvel_avg = int2fix15(0);
        neighboring_boids = 0;
        close_dx = int2fix15(0);
        close_dy = int2fix15(0);
        num_predators = 0;
        predator_dx = int2fix15(0);
        predator_dy = int2fix15(0);

        // erase boid
        drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, BLACK);
        // update boid's position and velocity
        boid_algo(&boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed, i) ;
        // draw the boid at its new position    
        // printf("%d\n", boids[i].vx);
        // printf("%d\n", boids[i].vy);
        drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, GREEN);
      }

      for (l = 0; l < curr_N_predators; l++)
      {
        // erase boid
        drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
        // update boid's position and velocity
        predator_algo(&boid0_turnfactor, &boid0_maxspeed, &boid0_minspeed, l) ;
        // draw the boid at its new position    
        // printf("%d\n", boids[i].vx);
        // printf("%d\n", boids[i].vy);
        drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, RED);
      }

      // draw the boundaries
      drawArena(should_draw) ;
 
        if (counter > 30) {
            spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

        //Display text: Number of boids, frame rate, time elapsed
      
        total_time = time_us_32() / 1000000;

        sprintf(str1, "Time Elapsed = %d seconds", total_time);

        sprintf(str2, "Spare Time = %d us", spare_time);

        sprintf(str3, "Frame Rate = %d us/frame", FRAME_RATE);

        sprintf(str4, "Number of boids = %d", curr_N_boids);

        fillRect(0, 0, 350, 50, BLACK);
        setCursor(10,10);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str1);

        setCursor(200,10);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str2);

        setCursor(10,30);
        setTextColor(WHITE);
        setTextSize(1);
        writeString(str3);

        setCursor(200,30);
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

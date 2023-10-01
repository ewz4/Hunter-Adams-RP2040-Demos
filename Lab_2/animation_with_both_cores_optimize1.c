
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

struct boid boids_0[N_boids];
struct boid boids_1[N_boids];
int curr_N_boids = 0;
volatile int i = 0;


struct predator {
    fix15 x ;
    fix15 y ;
    fix15 vx ;
    fix15 vy ;
};

#define N_predators 10
volatile int curr_N_predators = 0;
struct predator predators[N_predators];
volatile int k = 0;
volatile int l = 0;
fix15 predatory_range = int2fix15(100);
fix15 squared_predator_distance ; 
fix15 predatory_range_square = int2fix15(10000);
fix15 predator_dx ;
fix15 predator_dy ;
int num_predators = 0;
fix15 predator_turnfactor = float2fix15(0.5);
fix15 speed_predator ;

// fix15 distance_between_boids[45]; //45 combinations of boids

// Boid Variables
// fix15 boid0_turnfactor = float2fix15(0.2);
// fix15 boid0_visualRange = int2fix15(40);
// fix15 boid0_protectedRange = int2fix15(8);
// fix15 boid0_centeringfactor = float2fix15(0.0005);
// fix15 boid0_avoidfactor = float2fix15(0.05);
// fix15 boid0_matchingfactor = float2fix15(0.05);
// fix15 boid0_maxspeed = int2fix15(6);
// fix15 boid0_minspeed = int2fix15(3);
// fix15 boid0_protectedRangeSquared = int2fix15(64);
// fix15 boid0_visualRangeSquared = int2fix15(1600);
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = int2fix15(40);
fix15 protectedRange = int2fix15(8);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = int2fix15(6);
fix15 minspeed = int2fix15(3);
fix15 protectedRangeSquared = int2fix15(64);
fix15 visualRangeSquared = int2fix15(1600);


// Boid on Core 0
// accumulator vairables
fix15 xpos_avg_0 ;
fix15 ypos_avg_0 ;
fix15 xvel_avg_0 ;
fix15 yvel_avg_0 ;
int neighboring_boids_0 ;
fix15 close_dx_0 ;
fix15 close_dy_0 ;
// Needed for calc
fix15 speed_0 ;
fix15 dx_0 ;
fix15 dy_0 ;
fix15 squared_distance_0 ;
fix15 neighboring_boids_div_0 ;
int i_0 ;
fix15 squared_predator_distance_0 ;
fix15 predator_dx_0 ;
fix15 predator_dy_0 ;
int num_predators_0 ;

volatile int still_running = 1; // Flag to stop core 1 until core 0 is ready to move on


// Boid on core 1
fix15 xpos_avg_1 ;
fix15 ypos_avg_1 ;
fix15 xvel_avg_1 ;
fix15 yvel_avg_1 ;
int neighboring_boids_1 ;
fix15 close_dx_1 ;
fix15 close_dy_1 ;
// Needed for calc
fix15 speed_1 ;
fix15 dx_1 ;
fix15 dy_1 ;
fix15 squared_distance_1 ;
fix15 neighboring_boids_div_1 ;
int i_1 ;
fix15 squared_predator_distance_1 ;
fix15 predator_dx_1 ;
fix15 predator_dy_1 ;
int num_predators_1 ;

// fix15 boid1_x ;
// fix15 boid1_y ;
// fix15 boid1_vx ;
// fix15 boid1_vy ;

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
void spawnBoids(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
  *x = int2fix15(rand() % 640);
  *y = int2fix15(rand() % 480);
  *vx = int2fix15(rand() % 3 + 3);
  *vy = int2fix15(rand() % 3 + 3);
  // If running on Core 0
  // if (core == 0)
  // {
  //   for (i = 0; i < curr_N_boids/2; i++)
  //   {
  //     boids_0[i].x = int2fix15(rand() % 640);
  //     boids_0[i].y = int2fix15(rand() % 480);
  //     boids_0[i].vx = int2fix15(rand() % 3 + 3);
  //     boids_0[i].vy = int2fix15(rand() % 3 + 3);
  //   }
  //   for (k = 0; k < curr_N_predators; k++)
  //   {
  //     predators[k].x = int2fix15(rand() % 640);
  //     predators[k].y = int2fix15(rand() % 480);
  //     predators[k].vx = int2fix15(rand() % 3 + 3);
  //     predators[k].vy = int2fix15(rand() % 3 + 3);
  //   }
  //   k = 0;
  // }
  // else if (core == 1)
  // {
  //   for (i = 0; i < curr_N_boids/2; i++)
  //   {
  //     boids_1[i].x = int2fix15(rand() % 640);
  //     boids_1[i].y = int2fix15(rand() % 480);
  //     boids_1[i].vx = int2fix15(rand() % 3 + 3);
  //     boids_1[i].vy = int2fix15(rand() % 3 + 3);
  //   }
  // }
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
void boid_algo(int core)
{
  if (core == 0)
  {
    // For every other boid in the flock . . .
    for (int j = 0; j < curr_N_boids/2; j++) 
    {
      // For each boid other than the current boid
      if (j != i_0) 
      {
        // Compute differences in x and y coordinates
        dx_0 = boids_0[i_0].x - boids_0[j].x;
        dy_0 = boids_0[i_0].y - boids_0[j].y;
        // Are both those differences less than the visual range?
        if (absfix15(dx_0) < visualRange && absfix15(dy_0) < visualRange)
        {
          // If so, calculate the squared distance
          squared_distance_0 = multfix15(dx_0,dx_0) + multfix15(dy_0,dy_0);

          // Is squared distance less than the protected range?
          if (squared_distance_0 < protectedRangeSquared)
          {
              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx_0 += boids_0[i_0].x - boids_0[j].x;
              close_dy_0 += boids_0[i_0].y - boids_0[j].y;
          }
          // If not in protected range, is the boid in the visual range?
          else if (squared_distance_0 < visualRangeSquared)
          {
              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg_0 += boids_0[j].x;
              ypos_avg_0 += boids_0[j].y;
              xvel_avg_0 += boids_0[j].vx;
              yvel_avg_0 += boids_0[j].vy;
              
              // Increment number of boids within visual range
              neighboring_boids_0++;
          }
        }
      }
    }
    for (int j = 0; j < curr_N_boids/2; j++) 
    {
      // For each boid other than the current boid
      if (j != i_0) 
      {
        // Compute differences in x and y coordinates
        dx_0 = boids_0[i_0].x - boids_1[j].x;
        dy_0 = boids_0[i_0].y - boids_1[j].y;
        // Are both those differences less than the visual range?
        if (absfix15(dx_0) < visualRange && absfix15(dy_0) < visualRange)
        {
          // If so, calculate the squared distance
          squared_distance_0 = multfix15(dx_0,dx_0) + multfix15(dy_0,dy_0);

          // Is squared distance less than the protected range?
          if (squared_distance_0 < protectedRangeSquared)
          {
              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx_0 += boids_0[i_0].x - boids_1[j].x;
              close_dy_0 += boids_0[i_0].y - boids_1[j].y;
          }
          // If not in protected range, is the boid in the visual range?
          else if (squared_distance_0 < visualRangeSquared)
          {
              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg_0 += boids_1[j].x;
              ypos_avg_0 += boids_1[j].y;
              xvel_avg_0 += boids_1[j].vx;
              yvel_avg_0 += boids_1[j].vy;
              
              // Increment number of boids within visual range
              neighboring_boids_0++;
          }
        }
      }
    }

    // If there were any boids in the visual range
    if (neighboring_boids_0 > 0)
    {
      // Divide accumulator variables by number of boids in visual range
      neighboring_boids_div_0 = divfix(int2fix15(1), int2fix15(neighboring_boids_0));
      xpos_avg_0 = multfix15(xpos_avg_0,neighboring_boids_div_0);
      ypos_avg_0 = multfix15(ypos_avg_0,neighboring_boids_div_0);
      xvel_avg_0 = multfix15(xvel_avg_0,neighboring_boids_div_0);
      yvel_avg_0 = multfix15(yvel_avg_0,neighboring_boids_div_0);

      // Add the centering/matching contributions to velocity
      boids_0[i_0].vx = (boids_0[i_0].vx + 
                    multfix15(xpos_avg_0 - boids_0[i_0].x, centeringfactor) + 
                    multfix15(xvel_avg_0 - boids_0[i_0].vx, matchingfactor));
      boids_0[i_0].vy = (boids_0[i_0].vy + 
                    multfix15(ypos_avg_0 - boids_0[i_0].y, centeringfactor) + 
                    multfix15(yvel_avg_0 - boids_0[i_0].vy, matchingfactor));
    }

    // Add the avoidance contribution to velocity
    boids_0[i_0].vx = boids_0[i_0].vx + multfix15(close_dx_0, avoidfactor);
    boids_0[i_0].vy = boids_0[i_0].vy + multfix15(close_dy_0, avoidfactor);
    
    // If the boid is near an edge, make it turn by turnfactor
    // (this describes a box, will vary based on boundary conditions)
    if (should_draw == 0) //wrap everywhere
    {
      if (boids_0[i_0].y < int2fix15(y_screen_top))
      {
          boids_0[i_0].y = int2fix15(y_screen_bottom);
      }
      if (boids_0[i_0].y > int2fix15(y_screen_bottom))
      {
          boids_0[i_0].y = int2fix15(y_screen_top);
      } 
      if (boids_0[i_0].x < int2fix15(x_screen_left))
      {
          boids_0[i_0].x = int2fix15(x_screen_right);
      }
      if (boids_0[i_0].x > int2fix15(x_screen_right))
      {
          boids_0[i_0].x = int2fix15(x_screen_left);
      }  
    }
    else if (should_draw == 1) // if should_draw == 1 --> box
    {
      if (boids_0[i_0].y < int2fix15(y_margin_top_box))
      {
        boids_0[i_0].vy = boids_0[i_0].vy + turnfactor;
      }
      if (boids_0[i_0].y > int2fix15(y_margin_bottom_box))
      {
        boids_0[i].vy = boids_0[i_0].vy - turnfactor;
      } 
      if (boids_0[i_0].x < int2fix15(x_margin_left_box))
      {
        boids_0[i_0].vx = boids_0[i_0].vx + turnfactor;
      }
      if (boids_0[i_0].x > int2fix15(x_margin_right_box))
      {
        boids_0[i_0].vx = boids_0[i_0].vx - turnfactor;
      }  
    }
    else // should_draw == 2 --> draw 2 lines
    {
      if (boids_0[i_0].y < int2fix15(y_screen_top))
      {
          boids_0[i_0].y = int2fix15(y_screen_bottom);
      }
      if (boids_0[i_0].y > int2fix15(y_screen_bottom))
      {
          boids_0[i_0].y = int2fix15(y_screen_top);
      } 
      if (boids_0[i_0].x < int2fix15(x_margin_left_V_line))
      {
        boids_0[i_0].vx = boids_0[i_0].vx + turnfactor;
      }
      if (boids_0[i_0].x > int2fix15(x_margin_right_V_line))
      {
        boids_0[i_0].vx = boids_0[i_0].vx - turnfactor;
      }   
    }
  
    
    //////////////////////////////////
    // Predator Part
      
    for (k = 0; k < curr_N_predators; k++)
    {
      // Compute the differences in x and y coordinates
      dx_0 = boids_0[i_0].x - predators[k].x;
      dy_0 = boids_0[i_0].y - predators[k].y;

      // Are both those differences less than the predatory range?
      if (absfix15(dx_0) < predatory_range && absfix15(dy_0) < predatory_range)
      {
        // If so, calculate the squared distance to the predator
        squared_predator_distance_0 = multfix15(dx_0,dx_0) + multfix15(dy_0,dy_0);

        // Is the squared distance less than the predatory range squared?
        if (squared_predator_distance_0 < predatory_range_square)
        {
          predator_dx_0 += boids_0[i_0].x - predators[k].x;
          predator_dy_0 += boids_0[i_0].y - predators[k].y;

          // Increment the number of predators in the boid's predatory range
          num_predators_0 += 1;
        }
      }
    }

    // If there were any predators in the predatory range, turn away
    if (num_predators_0 > 0)
    {
      if (predator_dy_0 > 0)
      {
        boids_0[i_0].vy = boids_0[i_0].vy + predator_turnfactor;
      }
      if (predator_dy_0 < 0)
      {
        boids_0[i_0].vy = boids_0[i_0].vy - predator_turnfactor;
      }
      if (predator_dx > 0)
      {
        boids_0[i_0].vx = boids_0[i_0].vx + predator_turnfactor;
      }
      if (predator_dx < 0)
      {
        boids_0[i_0].vx = boids_0[i_0].vx - predator_turnfactor;
      }
    }
    //////////////////////////////////


    //Calculate the boid's speed
    //Slow step! Lookup the "alpha max plus beta min" algorithm  
    speed_0 = sqrtfix(multfix15(boids_0[i_0].vx,boids_0[i_0].vx) + 
                    multfix15(boids_0[i_0].vy,boids_0[i_0].vy));

    if (speed_0 > maxspeed) {
      boids_0[i_0].vx = multfix15(divfix(boids_0[i_0].vx, speed_0), maxspeed);
      boids_0[i_0].vy = multfix15(divfix(boids_0[i_0].vy, speed_0), maxspeed);
    }
    if (speed_0 < minspeed) {
      boids_0[i_0].vx = multfix15(divfix(boids_0[i_0].vx, speed_0), minspeed);
      boids_0[i_0].vy = multfix15(divfix(boids_0[i_0].vy, speed_0), minspeed);
    }

    // Update position using velocity
    boids_0[i_0].x = boids_0[i_0].x + boids_0[i_0].vx;
    boids_0[i_0].y = boids_0[i_0].y + boids_0[i_0].vy;
  }


  else if (core == 1)
  {
    // For every other boid in the flock . . .
    for (int j = 0; j < curr_N_boids/2; j++) 
    {
      // For each boid other than the current boid
      if (j != i_1) 
      {
        // Compute differences in x and y coordinates
        dx_1 = boids_1[i_1].x - boids_0[j].x;
        dy_1 = boids_1[i_1].y - boids_0[j].y;
        // Are both those differences less than the visual range?
        if (absfix15(dx_1) < visualRange && absfix15(dy_1) < visualRange)
        {
          // If so, calculate the squared distance
          squared_distance_1 = multfix15(dx_1,dx_1) + multfix15(dy_1,dy_1);

          // Is squared distance less than the protected range?
          if (squared_distance_1 < protectedRangeSquared)
          {
              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx_1 += boids_1[i_1].x - boids_0[j].x;
              close_dy_1 += boids_1[i_1].y - boids_0[j].y;
          }
          // If not in protected range, is the boid in the visual range?
          else if (squared_distance_1 < visualRangeSquared)
          {
              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg_1 += boids_0[j].x;
              ypos_avg_1 += boids_0[j].y;
              xvel_avg_1 += boids_0[j].vx;
              yvel_avg_1 += boids_0[j].vy;
              
              // Increment number of boids within visual range
              neighboring_boids_1++;
          }
        }
      }
    }
    for (int j = 0; j < curr_N_boids/2; j++) 
    {
      // For each boid other than the current boid
      if (j != i) 
      {
        // Compute differences in x and y coordinates
        dx_1 = boids_1[i_1].x - boids_1[j].x;
        dy_1 = boids_1[i_1].y - boids_1[j].y;
        // Are both those differences less than the visual range?
        if (absfix15(dx_1) < visualRange && absfix15(dy_1) < visualRange)
        {
          // If so, calculate the squared distance
          squared_distance_1 = multfix15(dx_1,dx_1) + multfix15(dy_1,dy_1);

          // Is squared distance less than the protected range?
          if (squared_distance_1 < protectedRangeSquared)
          {
              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx_1 += boids_1[i_1].x - boids_1[j].x;
              close_dy_1 += boids_1[i_1].y - boids_1[j].y;
          }
          // If not in protected range, is the boid in the visual range?
          else if (squared_distance_1 < visualRangeSquared)
          {
              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg_1 += boids_1[j].x;
              ypos_avg_1 += boids_1[j].y;
              xvel_avg_1 += boids_1[j].vx;
              yvel_avg_1 += boids_1[j].vy;
              
              // Increment number of boids within visual range
              neighboring_boids_1++;
          }
        }
      }
    }

    // If there were any boids in the visual range
    if (neighboring_boids_1 > 0)
    {
      // Divide accumulator variables by number of boids in visual range
      neighboring_boids_div_1 = divfix(int2fix15(1), int2fix15(neighboring_boids_1));
      xpos_avg_1 = multfix15(xpos_avg_1,neighboring_boids_div_1);
      ypos_avg_1 = multfix15(ypos_avg_1,neighboring_boids_div_1);
      xvel_avg_1 = multfix15(xvel_avg_1,neighboring_boids_div_1);
      yvel_avg_1 = multfix15(yvel_avg_1,neighboring_boids_div_1);

      // Add the centering/matching contributions to velocity
      boids_1[i_1].vx = (boids_1[i_1].vx + 
                    multfix15(xpos_avg_1 - boids_1[i_1].x, centeringfactor) + 
                    multfix15(xvel_avg_1 - boids_1[i_1].vx, matchingfactor));
      boids_1[i_1].vy = (boids_1[i_1].vy + 
                    multfix15(ypos_avg_1 - boids_1[i_1].y, centeringfactor) + 
                    multfix15(yvel_avg_1 - boids_1[i_1].vy, matchingfactor));
    }

    // Add the avoidance contribution to velocity
    boids_1[i_1].vx = boids_1[i_1].vx + multfix15(close_dx_1, avoidfactor);
    boids_1[i_1].vy = boids_1[i_1].vy + multfix15(close_dy_1, avoidfactor);
    
    // If the boid is near an edge, make it turn by turnfactor
    // (this describes a box, will vary based on boundary conditions)
    if (should_draw == 0) //wrap everywhere
    {
      if (boids_1[i_1].y < int2fix15(y_screen_top))
      {
          boids_1[i_1].y = int2fix15(y_screen_bottom);
      }
      if (boids_1[i_1].y > int2fix15(y_screen_bottom))
      {
          boids_1[i_1].y = int2fix15(y_screen_top);
      } 
      if (boids_1[i_1].x < int2fix15(x_screen_left))
      {
          boids_1[i_1].x = int2fix15(x_screen_right);
      }
      if (boids_1[i_1].x > int2fix15(x_screen_right))
      {
          boids_1[i_1].x = int2fix15(x_screen_left);
      }  
    }
    else if (should_draw == 1) // if should_draw == 1 --> box
    {
      if (boids_1[i_1].y < int2fix15(y_margin_top_box))
      {
        boids_1[i_1].vy = boids_1[i_1].vy + turnfactor;
      }
      if (boids_1[i_1].y > int2fix15(y_margin_bottom_box))
      {
        boids_1[i_1].vy = boids_1[i_1].vy - turnfactor;
      } 
      if (boids_1[i_1].x < int2fix15(x_margin_left_box))
      {
        boids_1[i_1].vx = boids_1[i_1].vx + turnfactor;
      }
      if (boids_1[i_1].x > int2fix15(x_margin_right_box))
      {
        boids_1[i_1].vx = boids_1[i_1].vx - turnfactor;
      }  
    }
    else // should_draw == 2 --> draw 2 lines
    {
      if (boids_1[i_1].y < int2fix15(y_screen_top))
      {
          boids_1[i_1].y = int2fix15(y_screen_bottom);
      }
      if (boids_1[i_1].y > int2fix15(y_screen_bottom))
      {
          boids_1[i_1].y = int2fix15(y_screen_top);
      } 
      if (boids_1[i_1].x < int2fix15(x_margin_left_V_line))
      {
        boids_1[i_1].vx = boids_1[i_1].vx + turnfactor;
      }
      if (boids_1[i_1].x > int2fix15(x_margin_right_V_line))
      {
        boids_1[i_1].vx = boids_1[i_1].vx - turnfactor;
      }   
    }
  
    
    //////////////////////////////////
    // Predator Part
      
    for (k = 0; k < curr_N_predators; k++)
    {
      
      // Compute the differences in x and y coordinates
      dx_1 = boids_1[i_1].x - predators[k].x;
      dy_1 = boids_1[i_1].y - predators[k].y;

      // Are both those differences less than the predatory range?
      if (absfix15(dx_1) < predatory_range && absfix15(dy_1) < predatory_range)
      {
        // If so, calculate the squared distance to the predator
        squared_predator_distance_1 = multfix15(dx_1,dx_1) + multfix15(dy_1,dy_1);

        // Is the squared distance less than the predatory range squared?
        if (squared_predator_distance_1 < predatory_range_square)
        {
          predator_dx_1 += boids_1[i_1].x - predators[k].x;
          predator_dy_1 += boids_1[i_1].y - predators[k].y;

          // Increment the number of predators in the boid's predatory range
          num_predators_1 += 1;
        }
      }
    }

    // If there were any predators in the predatory range, turn away
    if (num_predators_1 > 0)
    {
      if (predator_dy_1 > 0)
      {
        boids_1[i_1].vy = boids_0[i_1].vy + predator_turnfactor;
      }
      if (predator_dy_1 < 0)
      {
        boids_1[i_1].vy = boids_0[i_1].vy - predator_turnfactor;
      }
      if (predator_dx_1 > 0)
      {
        boids_1[i_1].vx = boids_0[i_1].vx + predator_turnfactor;
      }
      if (predator_dx_1 < 0)
      {
        boids_1[i_1].vx = boids_0[i_1].vx - predator_turnfactor;
      }
    }
    //////////////////////////////////


    //Calculate the boid's speed
    //Slow step! Lookup the "alpha max plus beta min" algorithm  
    speed_1 = sqrtfix(multfix15(boids_1[i_1].vx,boids_1[i_1].vx) + 
                    multfix15(boids_1[i_1].vy,boids_1[i_1].vy));

    if (speed_1 > maxspeed) {
      boids_1[i_1].vx = multfix15(divfix(boids_1[i_1].vx, speed_1), maxspeed);
      boids_1[i_1].vy = multfix15(divfix(boids_1[i_1].vy, speed_1), maxspeed);
    }
    if (speed_1 < minspeed) {
      boids_1[i_1].vx = multfix15(divfix(boids_1[i_1].vx, speed_1), minspeed);
      boids_1[i_1].vy = multfix15(divfix(boids_1[i_1].vy, speed_1), minspeed);
    }

    // Update position using velocity
    boids_1[i_1].x = boids_1[i_1].x + boids_1[i_1].vx;
    boids_1[i_1].y = boids_1[i_1].y + boids_1[i_1].vy;
  }
  
}

void predator_algo(fix15 speed, int l)
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
      predators[l].vy = predators[l].vy + turnfactor;
    }
    if (predators[l].y > int2fix15(y_margin_bottom_box))
    {
      predators[l].vy = predators[l].vy - turnfactor;
    } 
    if (predators[l].x < int2fix15(x_margin_left_box))
    {
      predators[l].vx = predators[l].vx + turnfactor;
    }
    if (predators[l].x > int2fix15(x_margin_right_box))
    {
      predators[l].vx = predators[l].vx - turnfactor;
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
      predators[l].vx = predators[l].vx + turnfactor;
    }
    if (predators[l].x > int2fix15(x_margin_right_V_line))
    {
      predators[l].vx = predators[l].vx - turnfactor;
    }   
  }
  //////////////////////////////////

  //Calculate the boid's speed
  //Slow step! Lookup the "alpha max plus beta min" algorithm  
  speed = sqrtfix(multfix15(predators[l].vx,predators[l].vx) + 
                  multfix15(predators[l].vy,predators[l].vy));

  if (speed > maxspeed) {
    predators[l].vx = multfix15(divfix(predators[l].vx, speed), maxspeed);
    predators[l].vy = multfix15(divfix(predators[l].vy, speed), maxspeed);
  }
  if (speed < minspeed) {
    predators[l].vx = multfix15(divfix(predators[l].vx, speed), minspeed);
    predators[l].vy = multfix15(divfix(predators[l].vy, speed), minspeed);
  }

  // Update position using velocity
  predators[l].x = predators[l].x + predators[l].vx;
  predators[l].y = predators[l].y + predators[l].vy;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
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
                turnfactor = float2fix15(atof(arg1));
            }
        }
        else if(strcmp(cmd,"visualrange")==0){
            if(arg1 != NULL){
                visualRange = int2fix15(atoi(arg1));
            }
        }
        else if(strcmp(cmd,"protectedrange")==0){
            if(arg1 != NULL){
                protectedRange = int2fix15(atoi(arg1));
            }
        }
        else if(strcmp(cmd,"centeringfactor")==0){
            if(arg1 != NULL){
                centeringfactor = float2fix15(atof(arg1));
            }
        }
        else if(strcmp(cmd,"numberBoids")==0){
            if(arg1 != NULL)
            {
              for (int i = 0; i < curr_N_boids/2; i++)
              {
                drawRect(fix2int15(boids_0[i].x), fix2int15(boids_0[i].y), 2, 2, BLACK);
                drawRect(fix2int15(boids_1[i].x), fix2int15(boids_1[i].y), 2, 2, BLACK);
              }

              for (l = 0; l < curr_N_predators; l++)
              {
                // erase boid
                drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
              }
              curr_N_boids = (int)(atof(arg1));
              for (i_0 = 0; i_0 < curr_N_boids/2; i_0++)
              {
                spawnBoids(&boids_0[i_0].x,&boids_0[i_0].y,&boids_0[i_0].vx,&boids_0[i_0].vy);
              }
              for (l = 0; l < curr_N_predators; l++)
              {
                spawnBoids(&predators[l].x,&predators[l].y,&predators[l].vx,&predators[l].vy);
              }
              for (i_1 = 0; i_1 < curr_N_boids/2; i_1++)
              {
                spawnBoids(&boids_1[i_0].x,&boids_1[i_0].y,&boids_1[i_0].vx,&boids_1[i_0].vy);
              }

            }
        }
        else if(strcmp(cmd,"numberPredators")==0){
            if(arg1 != NULL){
              for (int i = 0; i < curr_N_boids/2; i++)
              {
                drawRect(fix2int15(boids_0[i].x), fix2int15(boids_0[i].y), 2, 2, BLACK);
                drawRect(fix2int15(boids_1[i].x), fix2int15(boids_1[i].y), 2, 2, BLACK);
              }

              for (l = 0; l < curr_N_predators; l++)
              {
                // erase boid
                drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
              }
              
              curr_N_predators = (int)(atof(arg1));
              for (i_0 = 0; i_0 < curr_N_boids/2; i_0++)
              {
                spawnBoids(&boids_0[i_0].x,&boids_0[i_0].y,&boids_0[i_0].vx,&boids_0[i_0].vy);
              }
              for (l = 0; l < curr_N_predators; l++)
              {
                spawnBoids(&predators[l].x,&predators[l].y,&predators[l].vx,&predators[l].vy);
              }
              for (i_1 = 0; i_1 < curr_N_boids/2; i_1++)
              {
                spawnBoids(&boids_1[i_0].x,&boids_1[i_0].y,&boids_1[i_0].vx,&boids_1[i_0].vy);
              }
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
    static int begin_time_0 ;
    static int spare_time_0 ;
    static int total_time_0 = 0;
    static int counter_0 = 0;
    static int core_0 = 0;
    char str1[50];
    char str2[50];
    char str3[50];
    char str4[50];

    curr_N_boids = 100;
    curr_N_predators = 2;

    // Spawn a boid
    for (i_0 = 0; i_0 < curr_N_boids/2; i_0++)
    {
      spawnBoids(&boids_0[i_0].x,&boids_0[i_0].y,&boids_0[i_0].vx,&boids_0[i_0].vy);
    }
    for (l = 0; l < curr_N_predators; l++)
    {
      spawnBoids(&predators[l].x,&predators[l].y,&predators[l].vx,&predators[l].vy);
    }
    

    while(1) {
      // Measure time at start of thread
      begin_time_0 = time_us_32();  
      for (i_0 = 0; i_0 < curr_N_boids/2; i_0++)
      {
        // Zero all accumulator variables
        xpos_avg_0 = int2fix15(0);
        ypos_avg_0 = int2fix15(0);
        xvel_avg_0 = int2fix15(0);
        yvel_avg_0 = int2fix15(0);
        neighboring_boids_0 = 0;
        close_dx_0 = int2fix15(0);
        close_dy_0 = int2fix15(0);
        num_predators = 0;
        predator_dx = int2fix15(0);
        predator_dy = int2fix15(0);
        speed_predator = int2fix15(0);
        speed_0 = int2fix15(0);
        dx_0 = int2fix15(0);
        dy_0 = int2fix15(0);

        // erase boid
        drawRect(fix2int15(boids_0[i_0].x), fix2int15(boids_0[i_0].y), 2, 2, BLACK);
        // update boid's position and velocity
        boid_algo(core_0) ;
        // draw the boid at its new position    
        // printf("%d\n", boids[i].vx);
        // printf("%d\n", boids[i].vy);
        drawRect(fix2int15(boids_0[i_0].x), fix2int15(boids_0[i_0].y), 2, 2, 2);
      }

      for (l = 0; l < curr_N_predators; l++)
      {
        
        // erase boid
        drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
        // update boid's position and velocity
        predator_algo(speed_predator, l) ;
        // draw the boid at its new position    
        // printf("%d\n", boids[i].vx);
        // printf("%d\n", boids[i].vy);
        drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, 6);
      }

      // draw the boundaries
      drawArena(should_draw) ;
 
        if (counter_0 > 30) {
            spare_time_0 = FRAME_RATE - (time_us_32() - begin_time_0) ;

        //Display text: Number of boids, frame rate, time elapsed
      
        total_time_0 = time_us_32() / 1000000;

        sprintf(str1, "Time Elapsed=%ds", total_time_0);

        sprintf(str2, "Spare Time=%dus", spare_time_0);

        sprintf(str3, "Frame Rate=%dus/frame", FRAME_RATE);

        sprintf(str4, "# boids=%d", curr_N_boids);

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

        counter_0 = 0;
      }

      counter_0++;

      // yield for necessary amount of time
      PT_YIELD_usec(spare_time_0) ;
      // PT_YIELD_usec(1000000);
     // NEVER exit while
    } // END WHILE(1)
  still_running = 0;
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time_1 ;
    static int spare_time_1 ;
    static int total_time_1 = 0;
    static int counter_1 = 0;
    static int core_1 = 1;
    char str1[50];
    char str2[50];
    char str3[50];
    char str4[50];

    curr_N_boids = 100;

    // Spawn a boid
    for (i_1 = 0; i_1 < curr_N_boids/2; i_1++)
    {
      spawnBoids(&boids_1[i_0].x,&boids_1[i_0].y,&boids_1[i_0].vx,&boids_1[i_0].vy);
    }

    while(1) {
      // Measure time at start of thread
      begin_time_1 = time_us_32();  
      for (i_1 = 0; i_1 < curr_N_boids/2; i_1++)
      {
        // Zero all accumulator variables
        xpos_avg_1 = int2fix15(0);
        ypos_avg_1 = int2fix15(0);
        xvel_avg_1 = int2fix15(0);
        yvel_avg_1 = int2fix15(0);
        neighboring_boids_1 = 0;
        close_dx_1 = int2fix15(0);
        close_dy_1 = int2fix15(0);
        speed_1 = int2fix15(0);
        dx_1 = int2fix15(0);
        dy_1 = int2fix15(0);

        // erase boid
        drawRect(fix2int15(boids_1[i_1].x), fix2int15(boids_1[i_1].y), 2, 2, BLACK);
        // update boid's position and velocity
        boid_algo(core_1) ;
        // draw the boid at its new position    
        // printf("%d\n", boids[i].vx);
        // printf("%d\n", boids[i].vy);
        drawRect(fix2int15(boids_1[i_1].x), fix2int15(boids_1[i_1].y), 2, 2, 2);
      }
      // yield for necessary amount of time
      // PT_YIELD_usec(spare_time) ;
      // PT_YIELD_usec(1000000);
      // NEVER exit while
      while(still_running == 1)
      {

      }
      still_running = 1;
    } // END WHILE(1)
    
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 

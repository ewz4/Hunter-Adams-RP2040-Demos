
/**
 * Dual cores
 *
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

#include <stdint.h>

// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

// Wall detection
#define hitBottom(b) (b > int2fix15(380))
#define hitTop(b) (b < int2fix15(100))
#define hitLeft(a) (a < int2fix15(100))
#define hitRight(a) (a > int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE;

// Build boid and predator structs
struct boid
{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  fix15 close_dx_0;
  fix15 close_dy_0;
  fix15 xpos_avg_0;
  fix15 ypos_avg_0;
  fix15 xvel_avg_0;
  fix15 yvel_avg_0;
  uint16_t neighboring_boids_0;
  fix15 close_dx_1;
  fix15 close_dy_1;
  fix15 xpos_avg_1;
  fix15 ypos_avg_1;
  fix15 xvel_avg_1;
  fix15 yvel_avg_1;
  uint16_t neighboring_boids_1;
  fix15 predator_dx;
  fix15 predator_dy;
  uint8_t num_predators;
};

struct predator
{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
};

// Boid parameters
#define N_boids 1200
uint16_t curr_N_boids = 1000;
uint16_t half_N_boids = 500;
struct boid boids[N_boids];
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

// Predator parameters
#define N_predators 5
uint8_t curr_N_predators = 0;
struct predator predators[N_predators];
fix15 predatory_range = int2fix15(100);
fix15 predatory_range_squared = int2fix15(10000);
fix15 predator_turnfactor = float2fix15(0.5);

// All variables for both core stuff
volatile bool still_running_0_current_update = true;
volatile bool still_running_0_spawn = true;
volatile bool still_running_0_draw = true;
volatile bool still_running_1_current_update = true;
volatile bool still_running_1_spawn = true;
volatile bool still_running_1_draw = true;
volatile bool still_running_0_string_output = true;
volatile bool still_running_1_string_output = true;

// Margin Size
uint16_t x_margin_left_box = 100;
uint16_t x_margin_right_box = 540;
uint16_t x_change_margin_box = 440;
uint16_t y_margin_top_box = 100;
uint16_t y_margin_bottom_box = 380;
uint16_t y_change_margin_box = 280;
uint8_t should_draw = 1;

// Vertical Line Size
uint16_t x_margin_left_V_line = 200;
uint16_t x_margin_right_V_line = 440;
uint16_t y_margin_top_line = 0;
uint16_t y_change_margin_line = 480;

// Screen Size
uint16_t y_screen_top = 0;
uint16_t y_screen_bottom = 480;
uint16_t x_screen_left = 0;
uint16_t x_screen_right = 640;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Spawn boid or predator by assigning its position and velocity
void spawn(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
  *x = int2fix15(rand() % 640);
  *y = int2fix15(rand() % 480);
  *vx = int2fix15(rand() % 3 + 3);
  *vy = int2fix15(rand() % 3 + 3);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Draw the boundaries
void drawArena(int should_draw)
{
  if (should_draw == 1)
  {
    drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, WHITE);
    drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, WHITE);
    drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, WHITE);
    drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, WHITE);
  }
  else if (should_draw == 2)
  {
    drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, WHITE);
    drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, WHITE);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Boid_algo initial calculation for i_0 boid
void boid_algo_init_calc_core0(uint16_t i_0, uint16_t i_1)
{
  fix15 squared_distance_0;
  fix15 dx_i_0;
  fix15 dy_i_0;

  fix15 squared_predator_distance_0;
  fix15 dx_p_0;
  fix15 dy_p_0;

  for (uint16_t j = i_0 + 1; j < (i_1 + 1); j++)
  {
    dx_i_0 = boids[i_0].x - boids[j].x;
    dy_i_0 = boids[i_0].y - boids[j].y;
    // Are both those differences less than the visual range?
    if (absfix15(dx_i_0) < visualRange && absfix15(dy_i_0) < visualRange)
    {
      // If so, calculate the squared distance
      squared_distance_0 = multfix15(dx_i_0, dx_i_0) + multfix15(dy_i_0, dy_i_0);

      // Is squared distance less than the protected range?
      if (squared_distance_0 < protectedRangeSquared)
      {
        // If so, calculate difference in x/y-coordinates to nearfield boid
        boids[i_0].close_dx_0 += dx_i_0;
        boids[i_0].close_dy_0 += dy_i_0;
        boids[j].close_dx_0 -= dx_i_0;
        boids[j].close_dy_0 -= dy_i_0;
      }
      // If not in protected range, is the boid in the visual range?
      else if (squared_distance_0 < visualRangeSquared)
      {
        // Add other boid's x/y-coord and x/y vel to accumulator variables
        boids[i_0].xpos_avg_0 += boids[j].x;
        boids[i_0].ypos_avg_0 += boids[j].y;
        boids[i_0].xvel_avg_0 += boids[j].vx;
        boids[i_0].yvel_avg_0 += boids[j].vy;

        boids[j].xpos_avg_0 += boids[i_0].x;
        boids[j].ypos_avg_0 += boids[i_0].y;
        boids[j].xvel_avg_0 += boids[i_0].vx;
        boids[j].yvel_avg_0 += boids[i_0].vy;

        // Increment number of boids within visual range
        boids[i_0].neighboring_boids_0++;
        boids[j].neighboring_boids_0++;
      }
    }
  }
  for (uint8_t k = 0; k < curr_N_predators; k++)
  {
    // Compute the differences in x and y coordinates
    dx_p_0 = boids[i_0].x - predators[k].x;
    dy_p_0 = boids[i_0].y - predators[k].y;

    // Are both those differences less than the predatory range?
    if (absfix15(dx_p_0) < predatory_range && absfix15(dx_p_0) < predatory_range)
    {
      boids[i_0].predator_dx += boids[i_0].x - predators[k].x;
      boids[i_0].predator_dy += boids[i_0].y - predators[k].y;

      // Increment the number of predators in the boid's predatory range
      boids[i_0].num_predators++;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Boid_algo initial calculation for i_1 boid
void boid_algo_init_calc_core1(uint16_t i_0, uint16_t i_1)
{
  fix15 squared_distance_1;
  fix15 dx_i_1;
  fix15 dy_i_1;

  fix15 squared_predator_distance_1;
  fix15 dx_p_1;
  fix15 dy_p_1;

  for (uint16_t j = i_1 - 1; j > i_0; j--)
  {
    dx_i_1 = boids[i_1].x - boids[j].x;
    dy_i_1 = boids[i_1].y - boids[j].y;
    // Are both those differences less than the visual range?
    if (absfix15(dx_i_1) < visualRange && absfix15(dy_i_1) < visualRange)
    {
      // If so, calculate the squared distance
      squared_distance_1 = multfix15(dx_i_1, dx_i_1) + multfix15(dy_i_1, dy_i_1);

      // Is squared distance less than the protected range?
      if (squared_distance_1 < protectedRangeSquared)
      {
        // If so, calculate difference in x/y-coordinates to nearfield boid
        boids[i_1].close_dx_1 += dx_i_1;
        boids[i_1].close_dy_1 += dy_i_1;
        boids[j].close_dx_1 -= dx_i_1;
        boids[j].close_dy_1 -= dy_i_1;
      }
      // If not in protected range, is the boid in the visual range?
      else if (squared_distance_1 < visualRangeSquared)
      {
        // Add other boid's x/y-coord and x/y vel to accumulator variables
        boids[i_1].xpos_avg_1 += boids[j].x;
        boids[i_1].ypos_avg_1 += boids[j].y;
        boids[i_1].xvel_avg_1 += boids[j].vx;
        boids[i_1].yvel_avg_1 += boids[j].vy;

        boids[j].xpos_avg_1 += boids[i_1].x;
        boids[j].ypos_avg_1 += boids[i_1].y;
        boids[j].xvel_avg_1 += boids[i_1].vx;
        boids[j].yvel_avg_1 += boids[i_1].vy;

        // Increment number of boids within visual range
        boids[i_1].neighboring_boids_1++;
        boids[j].neighboring_boids_1++;
      }
    }
  }
  for (uint8_t k = 0; k < curr_N_predators; k++)
  {
    // Compute the differences in x and y coordinates
    dx_p_1 = boids[i_1].x - predators[k].x;
    dy_p_1 = boids[i_1].y - predators[k].y;

    // Are both those differences less than the predatory range?
    if (absfix15(dx_p_1) < predatory_range && absfix15(dx_p_1) < predatory_range)
    {
      boids[i_1].predator_dx += boids[i_1].x - predators[k].x;
      boids[i_1].predator_dy += boids[i_1].y - predators[k].y;

      // Increment the number of predators in the boid's predatory range
      boids[i_1].num_predators++;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Update the x and y positions of each boid
void boid_algo_update(uint16_t i_update)
{
  fix15 close_dx = boids[i_update].close_dx_0 + boids[i_update].close_dx_1;
  fix15 close_dy = boids[i_update].close_dy_0 + boids[i_update].close_dy_1;
  fix15 xpos_avg = boids[i_update].xpos_avg_0 + boids[i_update].xpos_avg_1;
  fix15 ypos_avg = boids[i_update].ypos_avg_0 + boids[i_update].ypos_avg_1;
  fix15 xvel_avg = boids[i_update].xvel_avg_0 + boids[i_update].xvel_avg_1;
  ;
  fix15 yvel_avg = boids[i_update].yvel_avg_0 + boids[i_update].yvel_avg_1;
  uint16_t neighboring_boids = boids[i_update].neighboring_boids_0 + boids[i_update].neighboring_boids_1;
  fix15 neighboring_boids_div;
  fix15 fin_xpos_avg;
  fix15 fin_ypos_avg;
  fix15 fin_xvel_avg;
  fix15 fin_yvel_avg;

  fix15 speed;
  // If there were any boids in the visual range
  if (neighboring_boids > 0)
  {
    // Divide accumulator variables by number of boids in visual range
    neighboring_boids_div = int2fix15(neighboring_boids);
    fin_xpos_avg = divfix(xpos_avg, neighboring_boids_div);
    fin_ypos_avg = divfix(ypos_avg, neighboring_boids_div);
    fin_xvel_avg = divfix(xvel_avg, neighboring_boids_div);
    fin_yvel_avg = divfix(yvel_avg, neighboring_boids_div);

    // Add the centering/matching contributions to velocity
    boids[i_update].vx = (boids[i_update].vx +
                          multfix15(fin_xpos_avg - boids[i_update].x, centeringfactor) +
                          multfix15(fin_xvel_avg - boids[i_update].vx, matchingfactor));
    boids[i_update].vy = (boids[i_update].vy +
                          multfix15(fin_ypos_avg - boids[i_update].y, centeringfactor) +
                          multfix15(fin_yvel_avg - boids[i_update].vy, matchingfactor));
  }

  // Add the avoidance contribution to velocity
  boids[i_update].vx = boids[i_update].vx + multfix15(close_dx, avoidfactor);
  boids[i_update].vy = boids[i_update].vy + multfix15(close_dy, avoidfactor);

  // If the boid is near box or lines, make it turn by turnfactor
  if (should_draw == 0) // no box or lines, wrap everywhere
  {
    if (boids[i_update].y < int2fix15(y_screen_top))
    {
      boids[i_update].y = int2fix15(y_screen_bottom);
    }
    if (boids[i_update].y > int2fix15(y_screen_bottom))
    {
      boids[i_update].y = int2fix15(y_screen_top);
    }
    if (boids[i_update].x < int2fix15(x_screen_left))
    {
      boids[i_update].x = int2fix15(x_screen_right);
    }
    if (boids[i_update].x > int2fix15(x_screen_right))
    {
      boids[i_update].x = int2fix15(x_screen_left);
    }
  }
  else if (should_draw == 1) // if should_draw == 1 --> box
  {
    if (boids[i_update].y < int2fix15(y_margin_top_box))
    {
      boids[i_update].vy = boids[i_update].vy + turnfactor;
    }
    if (boids[i_update].y > int2fix15(y_margin_bottom_box))
    {
      boids[i_update].vy = boids[i_update].vy - turnfactor;
    }
    if (boids[i_update].x < int2fix15(x_margin_left_box))
    {
      boids[i_update].vx = boids[i_update].vx + turnfactor;
    }
    if (boids[i_update].x > int2fix15(x_margin_right_box))
    {
      boids[i_update].vx = boids[i_update].vx - turnfactor;
    }
  }
  else // should_draw == 2 --> draw 2 lines
  {
    if (boids[i_update].y < int2fix15(y_screen_top))
    {
      boids[i_update].y = int2fix15(y_screen_bottom);
    }
    if (boids[i_update].y > int2fix15(y_screen_bottom))
    {
      boids[i_update].y = int2fix15(y_screen_top);
    }
    if (boids[i_update].x < int2fix15(x_margin_left_V_line))
    {
      boids[i_update].vx = boids[i_update].vx + turnfactor;
    }
    if (boids[i_update].x > int2fix15(x_margin_right_V_line))
    {
      boids[i_update].vx = boids[i_update].vx - turnfactor;
    }
  }

  // If there were any predators in the predatory range, turn away
  if (boids[i_update].num_predators > 0)
  {
    if (boids[i_update].predator_dy > 0)
    {
      boids[i_update].vy = boids[i_update].vy + predator_turnfactor;
    }
    if (boids[i_update].predator_dy < 0)
    {
      boids[i_update].vy = boids[i_update].vy - predator_turnfactor;
    }
    if (boids[i_update].predator_dx > 0)
    {
      boids[i_update].vx = boids[i_update].vx + predator_turnfactor;
    }
    if (boids[i_update].predator_dx < 0)
    {
      boids[i_update].vx = boids[i_update].vx - predator_turnfactor;
    }
  }
  //////////////////////////////////

  // Calculate the boid's speed
  // Slow step! Lookup the "alpha max plus beta min" algorithm
  if (absfix15(boids[i_update].vx) < absfix15(boids[i_update].vy))
  {
    speed = absfix15(boids[i_update].vy) + (absfix15(boids[i_update].vx) >> 2);
  }
  else
  {
    speed = absfix15(boids[i_update].vx) + (absfix15(boids[i_update].vy) >> 2);
  }

  // Can speed this up by multiplying by a fix constant instead of normalizing
  if (speed > maxspeed)
  {
    boids[i_update].vx = boids[i_update].vx - (boids[i_update].vx >> 2);
    boids[i_update].vy = boids[i_update].vy - (boids[i_update].vy >> 2);
  }
  if (speed < minspeed)
  {
    boids[i_update].vx = boids[i_update].vx + (boids[i_update].vx >> 2);
    boids[i_update].vy = boids[i_update].vy + (boids[i_update].vy >> 2);
  }

  // Update position using velocity
  boids[i_update].x = boids[i_update].x + boids[i_update].vx;
  boids[i_update].y = boids[i_update].y + boids[i_update].vy;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void predator_algo(uint8_t l)
{
  fix15 speed;
  // If the boid is near an edge, make it turn by turnfactor
  // (this describes a box, will vary based on boundary conditions)
  if (should_draw == 0) // wrap everywhere
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

  // Calculate the boid's speed
  // Slow step! Lookup the "alpha max plus beta min" algorithm
  if (absfix15(predators[l].vx) < absfix15(predators[l].vy))
  {
    speed = absfix15(predators[l].vy) + (absfix15(predators[l].vx) >> 2);
  }
  else
  {
    speed = absfix15(predators[l].vx) + (absfix15(predators[l].vy) >> 2);
  }

  if (speed > maxspeed)
  {
    predators[l].vx = predators[l].vx - (predators[l].vx >> 2);
    predators[l].vy = predators[l].vy - (predators[l].vy >> 2);
  }
  if (speed < minspeed)
  {
    predators[l].vx = predators[l].vx + (predators[l].vx >> 2);
    predators[l].vy = predators[l].vy + (predators[l].vy >> 2);
  }

  // Update position using velocity
  predators[l].x = predators[l].x + predators[l].vx;
  predators[l].y = predators[l].y + predators[l].vy;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);

  // wait for 1 sec
  PT_YIELD_usec(1000000);
  // announce the threader version
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write;

  static char cmd[16], arg1[6];
  static char *token;
  while (1)
  {
    // print prompt
    sprintf(pt_serial_out_buffer, "Enter Command> ");
    // spawn a thread to do the non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;

    //  tokenize
    token = strtok(pt_serial_in_buffer, " ");
    strcpy(cmd, token);
    token = strtok(NULL, " ");
    strcpy(arg1, token);

    // parse by command
    if (strcmp(cmd, "help") == 0)
    {
      // List commands
      printf("draw line\n\r");
      printf("draw box\n\r");
      printf("draw none\n\r");
      printf("turnfactor\n\r");
      printf("visualrange\n\r");
      printf("protectedrange\n\r");
      printf("centeringfactor\n\r");
      printf("avoidfactor\n\r");
      printf("matchingfactor\n\r");
      printf("numberBoids\n\r");
      printf("numberPredators\n\r");
    }
    else if (strcmp(cmd, "draw") == 0)
    {
      if (strcmp(arg1, "line") == 0)
      {
        should_draw = 2;
        drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, BLACK);
        drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, BLACK);
        drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, BLACK);
        drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, BLACK);
      }
      else if (strcmp(arg1, "box") == 0)
      {
        should_draw = 1;
        drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK);
        drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK);
      }
      else if (strcmp(arg1, "none") == 0)
      {
        should_draw = 0;
        drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, BLACK);
        drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, BLACK);
        drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, BLACK);
        drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, BLACK);

        drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, BLACK);
        drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, BLACK);
      }
    }
    //
    else if (strcmp(cmd, "turnfactor") == 0)
    {
      if (arg1 != NULL)
      {
        turnfactor = float2fix15(atof(arg1));
      }
    }
    else if (strcmp(cmd, "visualrange") == 0)
    {
      if (arg1 != NULL)
      {
        visualRange = int2fix15(atoi(arg1));
        visualRangeSquared = multfix15(visualRange, visualRange);
      }
    }
    else if (strcmp(cmd, "protectedrange") == 0)
    {
      if (arg1 != NULL)
      {
        protectedRange = int2fix15(atoi(arg1));
        protectedRangeSquared = multfix15(protectedRange, protectedRange);
      }
    }
    else if (strcmp(cmd, "centeringfactor") == 0)
    {
      if (arg1 != NULL)
      {
        centeringfactor = float2fix15(atof(arg1));
      }
    }
    else if (strcmp(cmd, "avoidfactor") == 0)
    {
      if (arg1 != NULL)
      {
        avoidfactor = float2fix15(atof(arg1));
      }
    }
    else if (strcmp(cmd, "matchingfactor") == 0)
    {
      if (arg1 != NULL)
      {
        matchingfactor = float2fix15(atof(arg1));
      }
    }
    else if (strcmp(cmd, "numberBoids") == 0)
    {
      if (arg1 != NULL)
      {
        for (uint16_t i = 0; i < curr_N_boids; i++)
        {
          drawPixel(fix2int15(boids[i].x), fix2int15(boids[i].y), BLACK);
        }

        for (uint8_t l = 0; l < curr_N_predators; l++)
        {
          // erase boid
          drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
        }
        curr_N_boids = (uint16_t)(atoi(arg1));
        half_N_boids = curr_N_boids >> 1;
        for (uint16_t i = 0; i < curr_N_boids; i++)
        {
          spawn(&boids[i].x, &boids[i].y, &boids[i].vx, &boids[i].vy);
        }
        for (uint8_t l = 0; l < curr_N_predators; l++)
        {
          spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
        }
      }
    }
    else if (strcmp(cmd, "numberPredators") == 0)
    {
      if (arg1 != NULL)
      {
        for (uint16_t i = 0; i < curr_N_boids; i++)
        {
          drawPixel(fix2int15(boids[i].x), fix2int15(boids[i].y), BLACK);
        }

        for (uint8_t l = 0; l < curr_N_predators; l++)
        {
          // erase boid
          drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
        }
        curr_N_predators = (uint8_t)(atoi(arg1));
        for (int i = 0; i < curr_N_boids; i++)
        {
          spawn(&boids[i].x, &boids[i].y, &boids[i].vx, &boids[i].vy);
        }
        for (int l = 0; l < curr_N_predators; l++)
        {
          spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
        }
      }
    }
    else printf("Huh?\n\r");
  } // END WHILE(1)
  PT_END(pt);
} // timer thread

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time_0;
  static int spare_time_0;
  static int total_time_0 = 0;
  static int counter_0 = 0;
  char str1[10];
  char str2[18];
  // char str3[25];
  char str4[11];

  for (uint16_t current_boid_0 = 0; current_boid_0 < half_N_boids; current_boid_0++)
  {
    spawn(&boids[current_boid_0].x, &boids[current_boid_0].y, &boids[current_boid_0].vx, &boids[current_boid_0].vy);
  }
  // Spawn all predators
  for (uint8_t l = 0; l < curr_N_predators; l++)
  {
    spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
  }

  // Wait for other thread to stop spawning
  still_running_0_spawn = false;
  while (still_running_1_spawn == true)
  {
  }
  still_running_1_spawn = true;

  while (1)
  {

    // Measure time at start of thread
    begin_time_0 = time_us_32();
    uint16_t current_boid_1 = curr_N_boids - 1;
    for (uint16_t current_boid_0 = 0; current_boid_0 < half_N_boids; current_boid_0++)
    {

      // update boid's position and velocity
      boid_algo_init_calc_core0(current_boid_0, current_boid_1);
      current_boid_1--;
    }

    still_running_0_current_update = false;
    // Wait until core 1 has finished updating
    while (still_running_1_current_update == true)
    {
    }
    still_running_1_current_update = true;

    for (uint16_t current_boid_0 = 0; current_boid_0 < half_N_boids; current_boid_0++)
    {
      // erase boid
      drawPixel(fix2int15(boids[current_boid_0].x), fix2int15(boids[current_boid_0].y), BLACK);
      // boid_combine_values(current_boid_0); // TODO: Move this to be inside boid_algo_update
      boid_algo_update(current_boid_0);
      // draw the boid at its new position
      drawPixel(fix2int15(boids[current_boid_0].x), fix2int15(boids[current_boid_0].y), WHITE);
      boids[current_boid_0].xpos_avg_0 = 0;
      boids[current_boid_0].ypos_avg_0 = 0;
      boids[current_boid_0].xvel_avg_0 = 0;
      boids[current_boid_0].yvel_avg_0 = 0;
      boids[current_boid_0].neighboring_boids_0 = 0;
      boids[current_boid_0].close_dx_0 = 0;
      boids[current_boid_0].close_dy_0 = 0;
      boids[current_boid_0].close_dy_0 = 0;
      boids[current_boid_0].xpos_avg_1 = 0;
      boids[current_boid_0].ypos_avg_1 = 0;
      boids[current_boid_0].xvel_avg_1 = 0;
      boids[current_boid_0].yvel_avg_1 = 0;
      boids[current_boid_0].neighboring_boids_1 = 0;
      boids[current_boid_0].close_dx_1 = 0;
      boids[current_boid_0].close_dy_1 = 0;
      boids[current_boid_0].num_predators = 0;
      boids[current_boid_0].predator_dx = 0;
      boids[current_boid_0].predator_dy = 0;
    }
    still_running_0_draw = false;
    while (still_running_1_draw == true)
    {
    }
    still_running_1_draw = true;

    for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
    {
      // erase predator
      drawRect(fix2int15(predators[current_predator].x), fix2int15(predators[current_predator].y), 2, 2, BLACK);
      // update boid's position and velocity
      predator_algo(current_predator);
      // draw the boid at its new position
      drawRect(fix2int15(predators[current_predator].x), fix2int15(predators[current_predator].y), 2, 2, RED);
    }

    // draw the boundaries
    drawArena(should_draw);
    // spare_time_0 = FRAME_RATE - (time_us_32() - begin_time_0);

    if (counter_0 > 30)
    {
      spare_time_0 = FRAME_RATE - (time_us_32() - begin_time_0);

      // Display text: Number of boids, frame rate, time elapsed

      total_time_0 = time_us_32() / 1000000;

      sprintf(str1, "Time=%d", total_time_0);

      sprintf(str2, "Spare Time=%d", spare_time_0);

      // sprintf(str3, "Frame Rate=%dus/frame", FRAME_RATE);

      sprintf(str4, "Boids=%d", curr_N_boids);

      fillRect(0, 0, 150, 70, BLACK);
      setCursor(10, 10);
      setTextColor(WHITE);
      setTextSize(1);
      writeString(str1);

      setCursor(10, 25);
      setTextColor(WHITE);
      setTextSize(1);
      writeString(str2);

      //   setCursor(10,40);
      //   setTextColor(WHITE);
      //   setTextSize(1);
      //   writeString(str3);

      setCursor(10, 40);
      setTextColor(WHITE);
      setTextSize(1);
      writeString(str4);

      counter_0 = 0;
    }

    counter_0++;

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time_0);
    still_running_0_string_output = false;
    while (still_running_1_string_output == true)
    {
    }
    still_running_1_string_output = true;
    // PT_YIELD_usec(1000000);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Animation on core 1
static PT_THREAD(protothread_anim1(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time_1;
  static int spare_time_1;

  for (uint16_t current_boid_1 = curr_N_boids - 1; current_boid_1 > half_N_boids - 1; current_boid_1--)
  {
    spawn(&boids[current_boid_1].x, &boids[current_boid_1].y, &boids[current_boid_1].vx, &boids[current_boid_1].vy);
  }
  still_running_1_spawn = false;
  while (still_running_0_spawn == true)
  {
  }
  still_running_0_spawn = true;

  while (1)
  {
    // Measure time at start of thread
    begin_time_1 = time_us_32();
    uint16_t current_boid_0 = 0;
    for (uint16_t current_boid_1 = curr_N_boids - 1; current_boid_1 > half_N_boids - 1; current_boid_1--)
    {

      // update boid's position and velocity
      boid_algo_init_calc_core1(current_boid_0, current_boid_1);

      current_boid_0++;
    }

    still_running_1_current_update = false;
    // Wait until core 0 has finished updating
    while (still_running_0_current_update == true)
    {
    }
    still_running_0_current_update = true;

    for (uint16_t current_boid_1 = curr_N_boids - 1; current_boid_1 > half_N_boids - 1; current_boid_1--)
    {
      // erase boid
      drawPixel(fix2int15(boids[current_boid_1].x), fix2int15(boids[current_boid_1].y), BLACK);

      // boid_combine_values(current_boid_1);
      boid_algo_update(current_boid_1);
      // draw the boid at its new position
      drawPixel(fix2int15(boids[current_boid_1].x), fix2int15(boids[current_boid_1].y), WHITE);
      boids[current_boid_1].xpos_avg_0 = 0;
      boids[current_boid_1].ypos_avg_0 = 0;
      boids[current_boid_1].xvel_avg_0 = 0;
      boids[current_boid_1].yvel_avg_0 = 0;
      boids[current_boid_1].neighboring_boids_0 = 0;
      boids[current_boid_1].close_dx_0 = 0;
      boids[current_boid_1].close_dy_0 = 0;
      boids[current_boid_1].xpos_avg_1 = 0;
      boids[current_boid_1].ypos_avg_1 = 0;
      boids[current_boid_1].xvel_avg_1 = 0;
      boids[current_boid_1].yvel_avg_1 = 0;
      boids[current_boid_1].neighboring_boids_1 = 0;
      boids[current_boid_1].close_dx_1 = 0;
      boids[current_boid_1].close_dy_1 = 0;
      boids[current_boid_1].num_predators = 0;
      boids[current_boid_1].predator_dx = 0;
      boids[current_boid_1].predator_dy = 0;
    }
    still_running_1_draw = false;

    // TODO: change this to the protothreads wait
    while (still_running_0_draw == true)
    {
    }
    still_running_0_draw = true;

    // draw the boundaries
    drawArena(should_draw);

    spare_time_1 = FRAME_RATE - (time_us_32() - begin_time_1);

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time_1);

    still_running_1_string_output = false;
    while (still_running_0_string_output == true)
    {
    }
    still_running_0_string_output = true;
    // PT_YIELD_usec(1000000);
    // NEVER exit while
  } // END WHILE(1)

  PT_END(pt);
} // animation thread

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  // set_sys_clock_khz(250000, true);
  // initialize stio
  stdio_init_all();

  // initialize VGA
  initVGA();

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start;
}

/**
 * Rafael Gottlieb (rdg244)
 * Eric Zhang (ewz4)
 *
 * Code adapted from Hunter Adams (vha3@cornell.edu)
 *
 * This code depicts the boids algorithm with a few optimizations and
 * cheats to maximize the flock size.
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
 *
 *
 */

// Include the VGA graphics library
#include "vga_graphics.h"

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h> // For different integer types

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

// The fixed point macros
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
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

// Boid and predator structs
struct boid
{
    // Current state of boid
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;

    // Variables of current state needed for boid algo calculation
    fix15 close_dx;
    fix15 close_dy;
    fix15 xpos_avg;
    fix15 ypos_avg;
    fix15 xvel_avg;
    fix15 yvel_avg;
    uint16_t neighboring_boids;
    fix15 predator_flock_dx;
    fix15 predator_flock_dy;
    uint16_t num_flock_predators;
    fix15 predator_dx;
    fix15 predator_dy;
    uint8_t num_predators;
};

uint8_t predator_spawn_index = 0;
struct predator
{
    // Current state of predators
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
    uint8_t alive_counter;
};

// Initializing boids
#define N_flocks 3
#define N_boids 100          // Max number of boids per flock
uint16_t curr_N_boids = 100; // Current number of boids
uint16_t half_N_boids = 50;
struct boid rock_flock[N_boids];    // Avoids paper flock
struct boid paper_flock[N_boids];   // Avoids scissor flock
struct boid scissor_flock[N_boids]; // Avoids rock flock

// Initializing boid parameters
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = int2fix15(40);
fix15 protectedRange = int2fix15(8);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = int2fix15(6);
fix15 minspeed = int2fix15(3);

// Initializing predatory flock parameters
fix15 predatory_flock_range = int2fix15(50);
fix15 predator_flock_turnfactor = float2fix15(0.5);

// Initializing predator s
#define N_predators 5         // Total # of possible predators
uint8_t curr_N_predators = 0; // Current # of predators
struct predator predators[N_predators];

// Initializing predator parameters
fix15 predatory_range = int2fix15(100);
fix15 predator_turnfactor = float2fix15(0.5);

// Mood
uint8_t mood = 0;

// All boolean values for both cores --> will wait for other core to synchronize animation
// volatile bool still_running_0_current_update = true;
// volatile bool still_running_0_spawn = true;
// volatile bool still_running_0_draw = true;
// volatile bool still_running_1_current_update = true;
// volatile bool still_running_1_spawn = true;
// volatile bool still_running_1_draw = true;
// volatile bool still_running_0_string_output = true;
// volatile bool still_running_1_string_output = true;

// Margin Size
// uint16_t x_margin_left_box = 100;
// uint16_t x_margin_right_box = 540;
// uint16_t x_change_margin_box = 440;
// uint16_t y_margin_top_box = 100;
// uint16_t y_margin_bottom_box = 380;
// uint16_t y_change_margin_box = 280;
// uint8_t should_draw = 1;

// Vertical Line Size
// uint16_t x_margin_left_V_line = 200;
// uint16_t x_margin_right_V_line = 440;
// uint16_t y_margin_top_line = 0;
// uint16_t y_change_margin_line = 480;

// Screen Size
// uint16_t y_screen_top = 0;
// uint16_t y_screen_bottom = 480;
// uint16_t x_screen_left = 0;
// uint16_t x_screen_right = 640;

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
// void drawArena(int should_draw)
// {
//     if (should_draw == 1)
//     {
//         // Draws a box on the screen
//         drawVLine(x_margin_left_box, y_margin_top_box, y_change_margin_box, WHITE);
//         drawVLine(x_margin_right_box, y_margin_top_box, y_change_margin_box, WHITE);
//         drawHLine(x_margin_left_box, y_margin_top_box, x_change_margin_box, WHITE);
//         drawHLine(x_margin_left_box, y_margin_bottom_box, x_change_margin_box, WHITE);
//     }
//     else if (should_draw == 2)
//     {
//         // Draws 2 vertical lines on the screen
//         drawVLine(x_margin_left_V_line, y_margin_top_line, y_change_margin_line, WHITE);
//         drawVLine(x_margin_right_V_line, y_margin_top_line, y_change_margin_line, WHITE);
//     }
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Boid_algo initial calculation for ith boid
void boid_algo_init_calc_core(uint16_t i, uint8_t flock_type)
{
    struct boid *curr_flock;
    struct boid *predator_flock;

    if (flock_type == 0)
    {
        curr_flock = rock_flock;
        predator_flock = paper_flock;
    }
    else if (flock_type == 1)
    {
        curr_flock = paper_flock;
        predator_flock = scissor_flock;
    }
    else if (flock_type == 2)
    {
        curr_flock = scissor_flock;
        predator_flock = rock_flock;
    }

    // Initializes values needed for each boid calculation
    fix15 squared_distance;
    fix15 dx_i;
    fix15 dy_i;

    fix15 squared_predator_distance;
    fix15 dx_p;
    fix15 dy_p;

    for (uint16_t j = i + 1; j < curr_N_boids; j++)
    {
        dx_i = curr_flock[i].x - curr_flock[j].x;
        dy_i = curr_flock[i].y - curr_flock[j].y;

        // Are both those differences less than the visual range?
        if (absfix15(dx_i) < visualRange && absfix15(dy_i) < visualRange)
        {
            // Are both those differences less than the protected range?
            if (absfix15(dx_i) < protectedRange && absfix15(dy_i) < protectedRange)
            {
                // If so, add dx and dy to close_dx and close_dy for current boid
                curr_flock[i].close_dx += dx_i;
                curr_flock[i].close_dy += dy_i;

                // If so, subtract dx and dy to close_dx and close_dy for other boid
                curr_flock[j].close_dx -= dx_i;
                curr_flock[j].close_dy -= dy_i;
            }
            else // Boid is in the visual range
            {
                // Add other boid's x/y-coord and x/y vel to accumulator variables to boids
                curr_flock[i].xpos_avg += curr_flock[j].x;
                curr_flock[i].ypos_avg += curr_flock[j].y;
                curr_flock[i].xvel_avg += curr_flock[j].vx;
                curr_flock[i].yvel_avg += curr_flock[j].vy;

                // Add boid's x/y-coord and x/y vel to accumulator variables to other boids
                curr_flock[j].xpos_avg += curr_flock[i].x;
                curr_flock[j].ypos_avg += curr_flock[i].y;
                curr_flock[j].xvel_avg += curr_flock[i].vx;
                curr_flock[j].yvel_avg += curr_flock[i].vy;

                // Increment number of boids within visual range to both the current and other boid
                curr_flock[i].neighboring_boids++;
                curr_flock[j].neighboring_boids++;
            }
        }
    }

    for (uint8_t k = 0; k < curr_N_boids; k++)
    {
        // Compute the differences in x and y coordinates
        dx_p = curr_flock[i].x - predator_flock[k].x;
        dy_p = curr_flock[i].y - predator_flock[k].y;

        // Are both those differences less than the predatory range?
        if (absfix15(dx_p) < predatory_flock_range && absfix15(dx_p) < predatory_flock_range)
        {
            curr_flock[i].predator_flock_dx += curr_flock[i].x - predator_flock[k].x;
            curr_flock[i].predator_flock_dy += curr_flock[i].y - predator_flock[k].y;

            // Increment the number of predators in the boid's predatory range
            curr_flock[i].num_flock_predators++;
        }
    }

    for (uint8_t k = 0; k < curr_N_predators; k++)
    {
        if (predators[k].alive_counter > 0)
        {
            // Compute the differences in x and y coordinates
            dx_p = curr_flock[i].x - predators[k].x;
            dy_p = curr_flock[i].y - predators[k].y;

            // Are both those differences less than the predatory range?
            if (absfix15(dx_p) < predatory_range && absfix15(dx_p) < predatory_range)
            {
                curr_flock[i].predator_dx += curr_flock[i].x - predators[k].x;
                curr_flock[i].predator_dy += curr_flock[i].y - predators[k].y;

                // Increment the number of predators in the boid's predatory range
                curr_flock[i].num_predators++;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Update the x and y positions of each boid
void boid_algo_update(uint16_t i_update, uint8_t flock_type)
{
    struct boid *curr_flock;
    struct boid *predator_flock;

    if (flock_type == 1)
    {
        curr_flock = rock_flock;
        predator_flock = paper_flock;
    }
    else if (flock_type == 2)
    {
        curr_flock = paper_flock;
        predator_flock = scissor_flock;
    }
    else if (flock_type == 3)
    {
        curr_flock = scissor_flock;
        predator_flock = rock_flock;
    }
    // Initializes values only needed for each boid update
    fix15 neighboring_boids_div;
    fix15 fin_xpos_avg;
    fix15 fin_ypos_avg;
    fix15 fin_xvel_avg;
    fix15 fin_yvel_avg;
    fix15 speed;

    // If there were any boids in the visual range
    if (curr_flock[i_update].neighboring_boids > 0)
    {
        // Divide accumulator variables by number of boids in visual range
        neighboring_boids_div = int2fix15(curr_flock[i_update].neighboring_boids);
        fin_xpos_avg = divfix(curr_flock[i_update].xpos_avg, neighboring_boids_div);
        fin_ypos_avg = divfix(curr_flock[i_update].ypos_avg, neighboring_boids_div);
        fin_xvel_avg = divfix(curr_flock[i_update].xvel_avg, neighboring_boids_div);
        fin_yvel_avg = divfix(curr_flock[i_update].yvel_avg, neighboring_boids_div);

        // Add the centering/matching contributions to velocity
        curr_flock[i_update].vx = (curr_flock[i_update].vx +
                                   multfix15(fin_xpos_avg - curr_flock[i_update].x, centeringfactor) +
                                   multfix15(fin_xvel_avg - curr_flock[i_update].vx, matchingfactor));
        curr_flock[i_update].vy = (curr_flock[i_update].vy +
                                   multfix15(fin_ypos_avg - curr_flock[i_update].y, centeringfactor) +
                                   multfix15(fin_yvel_avg - curr_flock[i_update].vy, matchingfactor));
    }

    // Add the avoidance contribution to velocity
    curr_flock[i_update].vx = curr_flock[i_update].vx + multfix15(curr_flock[i_update].close_dx, avoidfactor);
    curr_flock[i_update].vy = curr_flock[i_update].vy + multfix15(curr_flock[i_update].close_dy, avoidfactor);

    if (hitTop(curr_flock[i_update].y))
    {
        curr_flock[i_update].vy = curr_flock[i_update].vy + turnfactor;
    }
    else if (hitBottom(curr_flock[i_update].y))
    {
        curr_flock[i_update].vy = curr_flock[i_update].vy - turnfactor;
    }

    if (hitLeft(curr_flock[i_update].x))
    {
        curr_flock[i_update].vx = curr_flock[i_update].vx + turnfactor;
    }
    else if (hitRight(curr_flock[i_update].x))
    {
        curr_flock[i_update].vx = curr_flock[i_update].vx - turnfactor;
    }

    // If there were any predators from predatory flock in the flock predatory range, turn away
    if (curr_flock[i_update].num_flock_predators > 0)
    {
        if (curr_flock[i_update].predator_flock_dy > 0)
        {
            curr_flock[i_update].vy = curr_flock[i_update].vy + predator_flock_turnfactor;
        }
        if (curr_flock[i_update].predator_flock_dy < 0)
        {
            curr_flock[i_update].vy = curr_flock[i_update].vy - predator_flock_turnfactor;
        }
        if (curr_flock[i_update].predator_flock_dx > 0)
        {
            curr_flock[i_update].vx = curr_flock[i_update].vx + predator_flock_turnfactor;
        }
        if (curr_flock[i_update].predator_flock_dx < 0)
        {
            curr_flock[i_update].vx = curr_flock[i_update].vx - predator_flock_turnfactor;
        }
    }

    // If there were any predators in the predatory range, turn away
    if (curr_flock[i_update].num_predators > 0)
    {
        if (curr_flock[i_update].predator_dy > 0)
        {
            curr_flock[i_update].vy = curr_flock[i_update].vy + predator_turnfactor;
        }
        if (curr_flock[i_update].predator_dy < 0)
        {
            curr_flock[i_update].vy = curr_flock[i_update].vy - predator_turnfactor;
        }
        if (curr_flock[i_update].predator_dx > 0)
        {
            curr_flock[i_update].vx = curr_flock[i_update].vx + predator_turnfactor;
        }
        if (curr_flock[i_update].predator_dx < 0)
        {
            curr_flock[i_update].vx = curr_flock[i_update].vx - predator_turnfactor;
        }
    }

    // Calculate the boid's speed
    // Calculated using the alpha beta max algorithm
    // speed = 1*v_max + 1/4 * v_min --> shift by 2 instead of multiply 0.25
    if (absfix15(curr_flock[i_update].vx) < absfix15(curr_flock[i_update].vy))
    {

        speed = absfix15(curr_flock[i_update].vy) + (absfix15(curr_flock[i_update].vx) >> 2);
    }
    else
    {
        speed = absfix15(curr_flock[i_update].vx) + (absfix15(curr_flock[i_update].vy) >> 2);
    }

    if (speed > maxspeed)
    {
        curr_flock[i_update].vx = curr_flock[i_update].vx - (curr_flock[i_update].vx >> 2);
        curr_flock[i_update].vy = curr_flock[i_update].vy - (curr_flock[i_update].vy >> 2);
    }
    if (speed < minspeed)
    {
        curr_flock[i_update].vx = curr_flock[i_update].vx + (curr_flock[i_update].vx >> 2);
        curr_flock[i_update].vy = curr_flock[i_update].vy + (curr_flock[i_update].vy >> 2);
    }

    // Update position using velocity
    curr_flock[i_update].x = curr_flock[i_update].x + curr_flock[i_update].vx;
    curr_flock[i_update].y = curr_flock[i_update].y + curr_flock[i_update].vy;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void predator_algo(uint8_t l)
{

    fix15 speed;

    // If the predator is near a box edge, make it turn by turnfactor
    if (hitTop(predators[l].y))
    {
        predators[l].vy = predators[l].vy + turnfactor;
    }
    if (hitBottom(predators[l].y))
    {
        predators[l].vy = predators[l].vy - turnfactor;
    }
    if (hitLeft(predators[l].x))
    {
        predators[l].vx = predators[l].vx + turnfactor;
    }
    if (hitRight(predators[l].x))
    {
        predators[l].vx = predators[l].vx - turnfactor;
    }

    // Calculate the predator's speed
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

    // Update the alive counter
    predators[l].alive_counter++;
    if (predators[l].alive_counter > 5)
    {
        predators[l].alive_counter = 0;
    }
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
    // printf("Yoooooo\n\r");
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
            printf("turnfactor\n\r");
            printf("visualrange\n\r");
            printf("protectedrange\n\r");
            printf("centeringfactor\n\r");
            printf("avoidfactor\n\r");
            printf("matchingfactor\n\r");
            printf("numberBoids\n\r");
            printf("numberPredators\n\r");
            printf("numberBoids\n\r");
            printf("mood\n\r");
            printf("splash\n\r");
        }
        else if (strcmp(cmd,"from") == 0)
        {
            mood = (uint8_t)2;
        }
        // For each parameter, the serial monitor reads the parameter value and alters it
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
            }
        }
        else if (strcmp(cmd, "protectedrange") == 0)
        {
            if (arg1 != NULL)
            {
                protectedRange = int2fix15(atoi(arg1));
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
            // erase predators and boids, and rerandomize initialization
            if (arg1 != NULL)
            {
                for (uint8_t m = 0; m < N_flocks; m++)
                {
                    struct boid *curr_flock;
                    if (m == 1)
                    {
                        curr_flock = rock_flock;
                    }
                    else if (m == 2)
                    {
                        curr_flock = paper_flock;
                    }
                    else if (m == 3)
                    {
                        curr_flock = scissor_flock;
                    }

                    for (uint16_t i = 0; i < curr_N_boids; i++)
                    {
                        fillCircle(fix2int15(curr_flock[i].x), fix2int15(curr_flock[i].y), 2, BLACK);
                    }
                }

                for (uint8_t l = 0; l < curr_N_predators; l++)
                {
                    fillCircle(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, BLACK);
                }

                curr_N_boids = (uint16_t)(atoi(arg1));
                half_N_boids = curr_N_boids >> 1;

                // Spawn boid flocks
                for (uint8_t m = 0; m < N_flocks; m++)
                {
                    struct boid *curr_flock;
                    if (m == 1)
                    {
                        curr_flock = rock_flock;
                    }
                    else if (m == 2)
                    {
                        curr_flock = paper_flock;
                    }
                    else if (m == 3)
                    {
                        curr_flock = scissor_flock;
                    }

                    for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
                    {
                        spawn(&curr_flock[current_boid].x, &curr_flock[current_boid].y, &curr_flock[current_boid].vx, &curr_flock[current_boid].vy);
                    }
                }

                for (uint8_t l = 0; l < curr_N_predators; l++)
                {
                    spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
                }
            }
        }
        else if (strcmp(cmd, "numberPredators") == 0)
        {
            // erase predators and boids, and rerandomize initialization
            if (arg1 != NULL)
            {
                for (uint8_t m = 0; m < N_flocks; m++)
                {
                    struct boid *curr_flock;
                    if (m == 1)
                    {
                        curr_flock = rock_flock;
                    }
                    else if (m == 2)
                    {
                        curr_flock = paper_flock;
                    }
                    else if (m == 3)
                    {
                        curr_flock = scissor_flock;
                    }

                    for (uint16_t i = 0; i < curr_N_boids; i++)
                    {
                        fillCircle(fix2int15(curr_flock[i].x), fix2int15(curr_flock[i].y), 2, BLACK);
                    }
                }

                for (uint8_t l = 0; l < curr_N_predators; l++)
                {
                    fillCircle(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, BLACK);
                }

                curr_N_predators = (uint16_t)(atoi(arg1));

                // Spawn boid flocks
                for (uint8_t m = 0; m < N_flocks; m++)
                {
                    struct boid *curr_flock;
                    if (m == 1)
                    {
                        curr_flock = rock_flock;
                    }
                    else if (m == 2)
                    {
                        curr_flock = paper_flock;
                    }
                    else if (m == 3)
                    {
                        curr_flock = scissor_flock;
                    }

                    for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
                    {
                        spawn(&curr_flock[current_boid].x, &curr_flock[current_boid].y, &curr_flock[current_boid].vx, &curr_flock[current_boid].vy);
                    }
                }

                for (uint8_t l = 0; l < curr_N_predators; l++)
                {
                    spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
                }
            }
        }
        else if (strcmp(cmd, "mood") == 0)
        {
            if (arg1 != NULL)
            {
                mood = (uint8_t)(atoi(arg1));
            }
        }
        else if (strcmp(cmd, "splash") == 0)
        {
            if (arg1 != NULL)
            {
                predators[predator_spawn_index].alive_counter = 1;
                predator_spawn_index++;
                if (predator_spawn_index > 4)
                {
                    predator_spawn_index = 0;
                }
            }
        }
        else
            printf("Huh?\n\r");
    } // END WHILE(1)
    PT_END(pt);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    // printf("HERE\n");
    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;
    static int total_time;
    static int counter = 0;
    char str1[10];
    char str2[18];
    // char str3[25];
    char str4[11];

    // Spawn boid flocks
    for (uint8_t m = 0; m < N_flocks; m++)
    {
        printf("%d\n", m);
        struct boid *curr_flock;
        if (m == 0)
        {
            curr_flock = rock_flock;
        }
        else if (m == 1)
        {
            curr_flock = paper_flock;
        }
        else if (m == 2)
        {
            curr_flock = scissor_flock;
        }

        for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
        {
            // printf("%d\n", current_boid);
            spawn(&curr_flock[current_boid].x, &curr_flock[current_boid].y, &curr_flock[current_boid].vx, &curr_flock[current_boid].vy);
        }
    }

    // Spawn all predators
    for (uint8_t l = 0; l < curr_N_predators; l++)
    {
        spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
    }

    // printf("HERE\n");
    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();

        // printf("THREADING\n");

        for (uint8_t m = 0; m < N_flocks; m++)
        {
            for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
            {
                // update boid's position and velocity
                boid_algo_init_calc_core(current_boid, m);
            }
        }

        // printf("THREADING 2\n");

        char color = BLACK;
        // Calculate mood
        if (mood == 0)
        {
            color = GREEN;
        }
        else if (mood == 1)
        {
            color = BLUE;
        }
        else if (mood == 2)
        {
            color = RED;
        }

        // printf("THREADING 3\n");

        for (uint8_t m = 0; m < N_flocks; m++)
        {
            for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
            {
                struct boid *curr_flock;
                if (m == 0)
                {
                    curr_flock = rock_flock;
                }
                else if (m == 1)
                {
                    curr_flock = paper_flock;
                }
                else if (m == 2)
                {
                    curr_flock = scissor_flock;
                }

                // Erase boid
                fillCircle(fix2int15(curr_flock[current_boid].x), fix2int15(curr_flock[current_boid].y), 2, BLACK);

                // Update boid state
                boid_algo_update(current_boid, mood);

                // Draw the boid at its new position
                if (curr_flock[current_boid].num_predators > 0)
                {
                    fillCircle(fix2int15(curr_flock[current_boid].x), fix2int15(curr_flock[current_boid].y), 2, WHITE);
                }
                else
                {
                    fillCircle(fix2int15(curr_flock[current_boid].x), fix2int15(curr_flock[current_boid].y), 2, color);
                }

                // Set all values needed for boid calculate back to 0
                curr_flock[current_boid].close_dx = 0;
                curr_flock[current_boid].close_dy = 0;
                curr_flock[current_boid].xpos_avg = 0;
                curr_flock[current_boid].ypos_avg = 0;
                curr_flock[current_boid].xvel_avg = 0;
                curr_flock[current_boid].yvel_avg = 0;
                curr_flock[current_boid].neighboring_boids = 0;

                curr_flock[current_boid].predator_flock_dx = 0;
                curr_flock[current_boid].predator_flock_dy = 0;
                curr_flock[current_boid].num_flock_predators = 0;

                curr_flock[current_boid].predator_dx = 0;
                curr_flock[current_boid].predator_dy = 0;
                curr_flock[current_boid].num_predators = 0;
            }
        }

        // printf("THREADING 4\n");

        for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
        {
            // Erase predator
            fillCircle(fix2int15(predators[current_predator].x), fix2int15(predators[current_predator].y), 2, BLACK);
            // Update predator's position and velocity
            predator_algo(current_predator);

            if (predators[current_predator].alive_counter > 0)
            {
                // Draw the predator at its new position
                fillCircle(fix2int15(predators[current_predator].x), fix2int15(predators[current_predator].y), 2, WHITE);
            }
        }

        // printf("THREADING 5\n");

        if (counter > 30)
        {
            spare_time = FRAME_RATE - (time_us_32() - begin_time);

            // Display text on VGA display: Number of boids, frame rate, time elapsed

            total_time = time_us_32() / 1000000;
            sprintf(str1, "Time=%d", total_time);
            sprintf(str2, "Spare Time=%d", spare_time);
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

            setCursor(10, 40);
            setTextColor(WHITE);
            setTextSize(1);
            writeString(str4);

            counter = 0;
        }

        counter++;

        // Yield for necessary amount of time
        PT_YIELD_usec(spare_time);

        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
    set_sys_clock_khz(250000, true);
    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

    // Map LED to GPIO port, make it low
    //   gpio_init(LED);
    //   gpio_set_dir(LED, GPIO_OUT);
    //   gpio_put(LED, 0);

    // start core 1
    // multicore_reset_core1();
    // multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}

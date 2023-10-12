
/**
 * fix15 operations (starting point)
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

// Boid and predator structs
struct boid
{
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
};

struct predator
{
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
};

// Boid paramaeters
#define N_boids 600
struct boid boids[N_boids];
uint16_t curr_N_boids = 200;
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

// Boid attribute variables
fix15 speed;
fix15 dx;
fix15 dy;
fix15 squared_distance;
fix15 neighboring_boids_div;
fix15 xpos_avg;
fix15 ypos_avg;
fix15 xvel_avg;
fix15 yvel_avg;
uint16_t neighboring_boids;
uint8_t num_predators;
fix15 close_dx;
fix15 close_dy;

// Predator parameters
#define N_predators 10
uint8_t curr_N_predators = 0;
struct predator predators[N_predators];
fix15 predatory_range = int2fix15(100);
fix15 predatory_range_squared = int2fix15(10000);
fix15 predator_turnfactor = float2fix15(0.5);

// Predator attirbutes
fix15 squared_predator_distance;
fix15 predator_dx;
fix15 predator_dy;

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

// Spawn boid or predator by assigning its position and velocity
void spawn(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
    *x = int2fix15(rand() % 640);
    *y = int2fix15(rand() % 480);
    *vx = int2fix15(rand() % 3 + 3);
    *vy = int2fix15(rand() % 3 + 3);
}

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

// Detect wallstrikes, update velocity and position
void boid_algo(uint16_t i)
{
    //////////////////////////////////
    // Boids Part

    // For every other boid in the flock . . .
    for (uint16_t j = 0; j < curr_N_boids; j++)
    {
        // For each boid other than the current boid
        if (j != i)
        {
            // Compute differences in x and y coordinates
            dx = boids[i].x - boids[j].x;
            dy = boids[i].y - boids[j].y;

            // Are both those differences less than the visual range?
            if (absfix15(dx) < visualRange && absfix15(dy) < visualRange)
            {
                // If so, calculate the squared distance
                squared_distance = multfix15(dx, dx) + multfix15(dy, dy);

                // Is squared distance less than the protected range?
                if (squared_distance < protectedRangeSquared)
                {
                    // If so, calculate difference in x/y-coordinates to nearfield boid
                    close_dx += boids[i].x - boids[j].x;
                    close_dy += boids[i].y - boids[j].y;
                }
                // If not in protected range, is the boid in the visual range?
                else if (squared_distance < visualRangeSquared)
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
        neighboring_boids_div = int2fix15(neighboring_boids);
        xpos_avg = divfix(xpos_avg, neighboring_boids_div);
        ypos_avg = divfix(ypos_avg, neighboring_boids_div);
        xvel_avg = divfix(xvel_avg, neighboring_boids_div);
        yvel_avg = divfix(yvel_avg, neighboring_boids_div);

        // Add the centering/matching contributions to velocity
        boids[i].vx = (boids[i].vx +
                       multfix15(xpos_avg - boids[i].x, centeringfactor) +
                       multfix15(xvel_avg - boids[i].vx, matchingfactor));
        boids[i].vy = (boids[i].vy +
                       multfix15(ypos_avg - boids[i].y, centeringfactor) +
                       multfix15(yvel_avg - boids[i].vy, matchingfactor));
    }

    // Add the avoidance contribution to velocity
    boids[i].vx = boids[i].vx + multfix15(close_dx, avoidfactor);
    boids[i].vy = boids[i].vy + multfix15(close_dy, avoidfactor);

    // If the boid is near box or lines, make it turn by turnfactor
    if (should_draw == 0) // wrap everywhere
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
            boids[i].vy = boids[i].vy + turnfactor;
        }
        if (boids[i].y > int2fix15(y_margin_bottom_box))
        {
            boids[i].vy = boids[i].vy - turnfactor;
        }
        if (boids[i].x < int2fix15(x_margin_left_box))
        {
            boids[i].vx = boids[i].vx + turnfactor;
        }
        if (boids[i].x > int2fix15(x_margin_right_box))
        {
            boids[i].vx = boids[i].vx - turnfactor;
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
            boids[i].vx = boids[i].vx + turnfactor;
        }
        if (boids[i].x > int2fix15(x_margin_right_V_line))
        {
            boids[i].vx = boids[i].vx - turnfactor;
        }
    }

    //////////////////////////////////
    // Predator Part

    for (uint8_t k = 0; k < curr_N_predators; k++)
    {

        // Compute the differences in x and y coordinates
        dx = boids[i].x - predators[k].x;
        dy = boids[i].y - predators[k].y;

        // Are both those differences less than the predatory range?
        if (absfix15(dx) < predatory_range && absfix15(dy) < predatory_range)
        {
            // If so, calculate the squared distance to the predator
            squared_predator_distance = multfix15(dx, dx) + multfix15(dy, dy);

            // Is the squared distance less than the predatory range squared?
            if (squared_predator_distance < predatory_range_squared)
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

    // Calculate the boid's speed
    // Slow step! Lookup the "alpha max plus beta min" algorithm
    speed = sqrtfix(multfix15(boids[i].vx, boids[i].vx) +
                    multfix15(boids[i].vy, boids[i].vy));

    if (speed > maxspeed)
    {
        boids[i].vx = multfix15(divfix(boids[i].vx, speed), maxspeed);
        boids[i].vy = multfix15(divfix(boids[i].vy, speed), maxspeed);
    }
    if (speed < minspeed)
    {
        boids[i].vx = multfix15(divfix(boids[i].vx, speed), minspeed);
        boids[i].vy = multfix15(divfix(boids[i].vy, speed), minspeed);
    }

    // Update position using velocity
    boids[i].x = boids[i].x + boids[i].vx;
    boids[i].y = boids[i].y + boids[i].vy;
}

void predator_algo(int l)
{
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
    speed = sqrtfix(multfix15(predators[l].vx, predators[l].vx) +
                    multfix15(predators[l].vy, predators[l].vy));

    if (speed > maxspeed)
    {
        predators[l].vx = multfix15(divfix(predators[l].vx, speed), maxspeed);
        predators[l].vy = multfix15(divfix(predators[l].vy, speed), maxspeed);
    }
    if (speed < minspeed)
    {
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
static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);

    // wait for 1 sec
    PT_YIELD_usec(1000000);
    // announce the threader version

    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write;

    static char cmd[16], arg1[16];
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
                    drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
                }

                curr_N_boids = (uint16_t)(atoi(arg1));

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
    }
    else printf("Huh?\n\r");
} // END WHILE(1)
PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;
    static int total_time = 0;
    static int counter = 0;
    char str1[10];
    char str2[18];
    // char str3[25];
    char str4[11];

    // Spawn all boids
    for (uint16_t i = 0; i < curr_N_boids; i++)
    {
        spawn(&boids[i].x, &boids[i].y, &boids[i].vx, &boids[i].vy);
    }

    // Spawn all predators
    for (uint8_t l = 0; l < curr_N_predators; l++)
    {
        spawn(&predators[l].x, &predators[l].y, &predators[l].vx, &predators[l].vy);
    }

    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();
        for (uint16_t i = 0; i < curr_N_boids; i++)
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
            drawPixel(fix2int15(boids[i].x), fix2int15(boids[i].y), BLACK);
            // update boid's position and velocity
            boid_algo(i);
            // draw the boid at its new position
            drawPixel(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, WHITE);
        }

        for (uint8_t l = 0; l < curr_N_predators; l++)
        {
            // erase boid
            drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, BLACK);
            // update boid's position and velocity
            predator_algo(l);
            // draw the boid at its new position
            drawRect(fix2int15(predators[l].x), fix2int15(predators[l].y), 2, 2, RED);
        }

        // draw the boundaries
        drawArena(should_draw);

        if (counter > 30)
        {
            spare_time = FRAME_RATE - (time_us_32() - begin_time);

            // Display text: Number of boids, frame rate, time elapsed

            total_time = time_us_32() / 1000000;

            sprintf(str1, "Time Elapsed = %d seconds", total_time);

            sprintf(str2, "Spare Time = %d us", spare_time);

            sprintf(str4, "Number of boids = %d", curr_N_boids);

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

            counter = 0;
        }

        counter++;

        // yield for necessary amount of time
        PT_YIELD_usec(spare_time);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}

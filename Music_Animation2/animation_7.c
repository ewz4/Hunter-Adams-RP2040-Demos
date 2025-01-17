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
 *  - RP2040 GND ---> VGA GND, Audio GND
 *
 *  - GPIO 26 ---> Audio input [0-3.3V]
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - ADC channel 0
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 *
 */

// VGA graphics library
#include "vga256_graphics.h"

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
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

// protothreads header
#include "pt_cornell_rp2040_v1_1_1.h"

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////// Music Identification Variables ////////////////////

/////////////////////////// ADC configuration ////////////////////////////////
// ADC Channel and pin
#define ADC_CHAN 0
#define ADC_PIN 26
// Number of samples per FFT
#define NUM_SAMPLES 1024
// Number of samples per FFT, minus 1
#define NUM_SAMPLES_M_1 1023
// Length of short (16 bits) minus log2 number of samples (10)
#define SHIFT_AMOUNT 6
// Log2 number of samples
#define LOG2_NUM_SAMPLES 10
// Sample rate (Hz)
#define Fs 10000.0
// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

fix15 max_fr ;           // temporary variable for max freq calculation

// DMA channels for sampling ADC (VGA driver uses 0 and 1)
int sample_chan = 4;
int control_chan = 5;

// Max and min macros
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

// 0.4 in fixed point (used for alpha max plus beta min)
fix15 zero_point_4 = float2fix15(0.4);

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES];
// And here's where we'll copy those samples for FFT calculation
fix15 fr[NUM_SAMPLES];
fix15 fi[NUM_SAMPLES];

// Sine table for the FFT calculation
fix15 Sinewave[NUM_SAMPLES];
// Hann window table for FFT calculation
fix15 window[NUM_SAMPLES];

// Pointer to address of start of sample buffer
uint8_t *sample_address_pointer = &sample_array[0];

// Structure arrays
struct note_mag_freq_array // Struct that keeps mag and frequency of notes
{
    fix15 mag;
    fix15 freq;
};

struct note_mag_freq_mood_array // Struct that keeps mag and frequency of notes
{
    fix15 freq;
    float mood;
};

struct note_mag_freq_array current_loudest_3_notes[3]; // Take top 3 notes of current sample mag and freq
struct note_mag_freq_mood_array past_notes[20];     // Store past 10 high notes

// Global variables
volatile int overall_mood;
int counter_overall_mood = 0;
fix15 percent_diff = 0;
fix15 percent_diff_threshold = float2fix15(0.01);
fix15 old_note_mag = float2fix15(0.001);
fix15 freq_calc = float2fix15(Fs / NUM_SAMPLES);
fix15 percentage_high_note_diff = float2fix15(0.05);
fix15 mag_threshold = float2fix15(1.5);
volatile bool turn_on_predator = false;
int size_circle = 2;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////// Animation Variables ////////////////////

// Wall detection
#define hitBottom(b) (b > int2fix15(180))
#define hitTop(b) (b < int2fix15(60))
#define hitLeft(a) (a < int2fix15(60))
#define hitRight(a) (a > int2fix15(260))

// uS per frame
#define FRAME_RATE 120000

// Boid and predator structs
struct boid
{
    // Current state of boid
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
    int hue;

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
    int hue;
};

// Initializing boids
#define N_flocks 3
#define N_boids 400              // Max number of boids per flock
uint16_t curr_N_boids = 300;     // Current number of boids
struct boid boid_flock[N_boids]; // Avoids paper flock

// Initializing boid parameters
fix15 turnfactor = float2fix15(0.3);
fix15 visualRange = int2fix15(40);
fix15 protectedRange = int2fix15(10);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.25);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = int2fix15(4);
fix15 minspeed = int2fix15(1);

// Initializing predatory flock parameters
fix15 predator_flock_range = int2fix15(5);
fix15 predator_flock_turnfactor = float2fix15(0.3);
fix15 maxspeed_predators = int2fix15(6);
fix15 minspeed_predators = int2fix15(3);
fix15 turnfactor_predators = float2fix15(0.5);

// Initializing predator s
#define N_predators 5         // Total # of possible predators
volatile uint8_t curr_N_predators = 3; // Current # of predators
struct predator predators[N_predators];

// Initializing predator parameters
fix15 predator_range = int2fix15(75);
fix15 predator_turnfactor = float2fix15(1.5);

bool change_to_overall = false;
int counter_hue = 0;


struct tile
{
    // Current state of predators
    int x;
    int y;
    int total_hue;
    int num_boids;
};

#define MAX_TILES 192
// #define SCREEN_WIDTH 320
// #define SCREEN_HEIGHT 240
int width = 16;  // tiles wide
int height = 12; // tiles wide
int total_tiles = 192;
int tile_side = 20;

struct tile tiles[MAX_TILES];

// Variables shared by cores

uint8_t mood = 0;
volatile float Splash_Color = 0;
volatile int animate_mood_1;
volatile int animate_mood_2;
volatile int animate_mood_3;


// Debugging
float error = 0;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////// Music Identification Functions /////////////////////

int solve_for_cents(fix15 a, fix15 b)
{
    // This funciton solves for the logarithmic ratio of frequencies using the function:
    // cents = 1200 * log_2(f2/f1)
    // Function identifies the musical interval
    float freq_ratio;
    float log2;
    int cents;
    freq_ratio = fix2float15(divfix(b, a));
    log2 = log10(freq_ratio) / log10(2);
    cents = (int)(12 * log2); // Cents is by every 100, so divide by 100 to get smaller int value

    return cents;
}

int identify_music_mood(int cents)
{
    // This function takes the calculated interval and determines the mood (color) of the interval
    int color_degree;
    // If the interval is greater than an octave, subtract until within range. Is the same relative interval
    while (cents > 12)
    {
        cents -= 12;
    }

    // Unison, Major Third, Perfect Fourth, Perfect Fifth
    if (cents == 0 || cents == 4 || cents == 5 || cents == 7)
    { // Major = Light green
        color_degree = 150;
    }
    // Major Second, Minor Third, Minor Sixth, Major Sixth, Octave
    else if (cents == 2 || cents == 3 || cents == 8 || cents == 9 || cents == 12)
    { // Minor = Blue
        color_degree = 240;
    }
    // Minor second, Tritone, Minor Seventh, Major Seventh
    else if (cents == 1 || cents == 6 || cents == 10 || cents == 11)
    { // Dissonant = Red
        color_degree = 359;
    }
    return color_degree;
}

int music_stuff()
{

    fix15 percentage_high_note_2;
    fix15 percentage_high_note_3;
    int cents_with_prev_note;
    fix15 top_note = 0;
    fix15 middle_note = 0;
    fix15 bottom_note = 0;
    int interval_1;
    int interval_2;
    int interval_3;
    int sum_mood = 0;
    int num_new_colors;

    // Determines if the second and third highest notes are close enough in amplitude to the loudest note 
    // to see if it is a note deliberately played
    percentage_high_note_2 = divfix(current_loudest_3_notes[1].mag - current_loudest_3_notes[0].mag, current_loudest_3_notes[0].mag);
    percentage_high_note_3 = divfix(current_loudest_3_notes[2].mag - current_loudest_3_notes[0].mag, current_loudest_3_notes[0].mag);

    if (abs(percentage_high_note_2) < percentage_high_note_diff && abs(percentage_high_note_3) < percentage_high_note_diff)
    {
        // All 3 notes, find the loudest frequency and calculate intervals and colors
        for (int m = 0; m < 3; m++)
        {
            if (current_loudest_3_notes[m].freq > top_note)
                top_note = current_loudest_3_notes[m].freq;
            else if (current_loudest_3_notes[m].freq > middle_note)
                middle_note = current_loudest_3_notes[m].freq;
            else
                bottom_note = current_loudest_3_notes[m].freq;
        }
        interval_1 = solve_for_cents(past_notes[9].freq, top_note);
        interval_2 = solve_for_cents(bottom_note, middle_note);
        interval_3 = solve_for_cents(middle_note, top_note);
        animate_mood_1 = identify_music_mood(interval_1);
        animate_mood_2 = identify_music_mood(interval_2);
        animate_mood_3 = identify_music_mood(interval_3);
        num_new_colors = 3;
    }
    else if (abs(percentage_high_note_2) < percentage_high_note_diff)
    {
        // Only notes 0 and 1, find the loudest frequency and calculate intervals and colors
        if (current_loudest_3_notes[0].freq > current_loudest_3_notes[1].freq)
        {
            top_note = current_loudest_3_notes[0].freq;
            middle_note = current_loudest_3_notes[1].freq;
        }
        else
        {
            top_note = current_loudest_3_notes[1].freq;
            middle_note = current_loudest_3_notes[0].freq;
        }
        interval_1 = solve_for_cents(past_notes[9].freq, top_note);
        interval_2 = solve_for_cents(bottom_note, top_note);
        animate_mood_1 = identify_music_mood(interval_1);
        animate_mood_2 = identify_music_mood(interval_2);
        num_new_colors = 2;
    }
    else if (abs(percentage_high_note_3) < percentage_high_note_diff)
    {
        // Only notes 0 and 2, find the loudest frequency and calculate intervals and colors
        if (current_loudest_3_notes[0].freq > current_loudest_3_notes[2].freq)
        {
            top_note = current_loudest_3_notes[0].freq;
            middle_note = current_loudest_3_notes[2].freq;
        }
        else
        {
            top_note = current_loudest_3_notes[2].freq;
            middle_note = current_loudest_3_notes[0].freq;
        }
        interval_1 = solve_for_cents(past_notes[9].freq, top_note);
        interval_2 = solve_for_cents(middle_note, top_note);
        animate_mood_1 = identify_music_mood(interval_1);
        animate_mood_2 = identify_music_mood(interval_2);
        num_new_colors = 2;
    }
    else
    {
        // Only the loudest note, calculate interval and color
        top_note = current_loudest_3_notes[0].freq;
        cents_with_prev_note = solve_for_cents(past_notes[9].freq, top_note);
        animate_mood_1 = identify_music_mood(cents_with_prev_note);
        num_new_colors = 1;
    }

    // Overall mood is recalculated every 20 notes
    if (counter_overall_mood > 20)
    {
        for (int i = 0; i < 20; i++)
        {
            if (i == 19)
            {
                past_notes[i].freq = top_note;
                past_notes[i].mood = animate_mood_1;
                sum_mood += past_notes[i].mood;
            }
            else
            {
                past_notes[i].freq = past_notes[i + 1].freq;
                past_notes[i].mood = past_notes[i + 1].mood;
                sum_mood += past_notes[i].mood;
            }
        }
        // printf("sum_mood = %f\n",sum_mood);
        overall_mood = sum_mood / 20;
        counter_overall_mood = 0;
    }
    else counter_overall_mood++;
    
    return num_new_colors;
}

// Peforms an in-place FFT. For more information about how this
// algorithm works, please see https://vanhunteradams.com/FFT/FFT.html
void FFTfix(fix15 fr[], fix15 fi[])
{

    unsigned short m;  // one of the indices being swapped
    unsigned short mr; // the other index being swapped (r for reversed)
    fix15 tr, ti;      // for temporary storage while swapping, and during iteration

    int i, j; // indices being combined in Danielson-Lanczos part of the algorithm
    int L;    // length of the FFT's being combined
    int k;    // used for looking up trig values from sine table

    int istep; // length of the FFT which results from combining two FFT's

    fix15 wr, wi; // trigonometric values from lookup table
    fix15 qr, qi; // temporary variables used during DL part of the algorithm

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here:
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    for (m = 1; m < NUM_SAMPLES_M_1; m++)
    {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ...
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT;
        // don't swap that which has already been swapped
        if (mr <= m)
            continue;
        // swap the bit-reveresed indices
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
        ti = fi[m];
        fi[m] = fi[mr];
        fi[mr] = ti;
    }
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1;
    // Log2 of number of samples, minus 1
    k = LOG2_NUM_SAMPLES - 1;
    // While the length of the FFT's being combined is less than the number
    // of gathered samples . . .
    while (L < NUM_SAMPLES)
    {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L << 1;
        // For each element in the FFT's that are being combined . . .
        for (m = 0; m < L; ++m)
        {
            // Lookup the trig values for that element
            j = m << k;                         // index of the sine table
            wr = Sinewave[j + NUM_SAMPLES / 4]; // cos(2pi m/N)
            wi = -Sinewave[j];                  // sin(2pi m/N)
            wr >>= 1;                           // divide by two
            wi >>= 1;                           // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i = m; i < NUM_SAMPLES; i += istep)
            {
                // j gets the index of the FFT element being combined with i
                j = i + L;
                // compute the trig terms (bottom half of the above matrix)
                tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]);
                ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]);
                // divide ith index elements by two (top half of above matrix)
                qr = fr[i] >> 1;
                qi = fi[i] >> 1;
                // compute the new values at each index
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            }
        }
        --k;
        L = istep;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////// Animation Functions ///////////////////////

// Spawn boid or predator by assigning its position and velocity
void spawn(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
    *x = int2fix15(rand() % 320);
    *y = int2fix15(rand() % 240);
    *vx = int2fix15(rand() % 3 + 3);
    *vy = int2fix15(rand() % 3 + 3);
}

// Boid_algo initial calculation for ith boid
void boid_algo_init_calc(uint16_t curr_boid)
{
    int difference_from_overall_color;

    for (uint16_t i = curr_boid + 1; i < curr_N_boids; i++)
    {
        fix15 dx_i = boid_flock[curr_boid].x - boid_flock[i].x;
        fix15 dy_i = boid_flock[curr_boid].y - boid_flock[i].y;

        // Are both those differences less than the visual range?
        if (absfix15(dx_i) < visualRange && absfix15(dy_i) < visualRange)
        {
            // Are both those differences less than the protected range?
            if (absfix15(dx_i) < protectedRange && absfix15(dy_i) < protectedRange)
            {
                // If so, add dx and dy to close_dx and close_dy for current boid
                boid_flock[curr_boid].close_dx += dx_i;
                boid_flock[curr_boid].close_dy += dy_i;

                // If so, subtract dx and dy to close_dx and close_dy for other boid
                boid_flock[i].close_dx -= dx_i;
                boid_flock[i].close_dy -= dy_i;
            }
            else // Boid is in the visual range
            {
                // Add other boid's x/y-coord and x/y vel to accumulator variables to boids
                boid_flock[curr_boid].xpos_avg += boid_flock[i].x;
                boid_flock[curr_boid].ypos_avg += boid_flock[i].y;
                boid_flock[curr_boid].xvel_avg += boid_flock[i].vx;
                boid_flock[curr_boid].yvel_avg += boid_flock[i].vy;

                // Add boid's x/y-coord and x/y vel to accumulator variables to other boids
                boid_flock[i].xpos_avg += boid_flock[curr_boid].x;
                boid_flock[i].ypos_avg += boid_flock[curr_boid].y;
                boid_flock[i].xvel_avg += boid_flock[curr_boid].vx;
                boid_flock[i].yvel_avg += boid_flock[curr_boid].vy;

                // Increment number of boids within visual range to both the current and other boid
                boid_flock[curr_boid].neighboring_boids++;
                boid_flock[i].neighboring_boids++;
            }
        }
    }

    if (change_to_overall)
    {
        // overall_mood is a hue
        difference_from_overall_color = overall_mood - boid_flock[curr_boid].hue;
        if (difference_from_overall_color > 0)
            boid_flock[curr_boid].hue += 15;
        else if (difference_from_overall_color < 0)
            boid_flock[curr_boid].hue -= 15;
    }

    if (turn_on_predator)
    {
        for (uint8_t i = 0; i < curr_N_predators; i++)
        {
            // Compute the differences in x and y coordinates
            fix15 dx_p = boid_flock[curr_boid].x - predators[i].x;
            fix15 dy_p = boid_flock[curr_boid].y - predators[i].y;

            // Are both those differences less than the predatory range?
            if (absfix15(dx_p) < predator_range && absfix15(dy_p) < predator_range)
            {
                boid_flock[curr_boid].predator_dx += boid_flock[curr_boid].x - predators[i].x;
                boid_flock[curr_boid].predator_dy += boid_flock[curr_boid].y - predators[i].y;

                if (i = 0)
                {
                    boid_flock[curr_boid].hue = predators[i].hue;
                }
                else
                {
                    boid_flock[curr_boid].hue += predators[i].hue;
                }

                // Increment the number of predators in the boid's predatory range
                boid_flock[curr_boid].num_predators++;
            }
        }

        if (boid_flock[curr_boid].num_predators > 0)
        {
            boid_flock[curr_boid].hue = boid_flock[curr_boid].hue / (int)boid_flock[curr_boid].num_predators;
        }
    }

    while (boid_flock[curr_boid].hue >= 360)
    {
        boid_flock[curr_boid].hue -= 360;
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Update the x and y positions of each boid
void boid_algo_update(uint16_t curr_boid)
{
    // If there were any boids in the visual range
    if (boid_flock[curr_boid].neighboring_boids > 0)
    {
        // Divide accumulator variables by number of boids in visual range
        fix15 neighboring_boids_div = int2fix15(boid_flock[curr_boid].neighboring_boids);
        fix15 fin_xpos_avg = divfix(boid_flock[curr_boid].xpos_avg, neighboring_boids_div);
        fix15 fin_ypos_avg = divfix(boid_flock[curr_boid].ypos_avg, neighboring_boids_div);
        fix15 fin_xvel_avg = divfix(boid_flock[curr_boid].xvel_avg, neighboring_boids_div);
        fix15 fin_yvel_avg = divfix(boid_flock[curr_boid].yvel_avg, neighboring_boids_div);

        // Add the centering/matching contributions to velocity
        boid_flock[curr_boid].vx = (boid_flock[curr_boid].vx +
                                    multfix15(fin_xpos_avg - boid_flock[curr_boid].x, centeringfactor) +
                                    multfix15(fin_xvel_avg - boid_flock[curr_boid].vx, matchingfactor));
        boid_flock[curr_boid].vy = (boid_flock[curr_boid].vy +
                                    multfix15(fin_ypos_avg - boid_flock[curr_boid].y, centeringfactor) +
                                    multfix15(fin_yvel_avg - boid_flock[curr_boid].vy, matchingfactor));
    }

    // Add the avoidance contribution to velocity
    boid_flock[curr_boid].vx = boid_flock[curr_boid].vx + multfix15(boid_flock[curr_boid].close_dx, avoidfactor);
    boid_flock[curr_boid].vy = boid_flock[curr_boid].vy + multfix15(boid_flock[curr_boid].close_dy, avoidfactor);

    // Add box arena contribution to velocity
    if (hitTop(boid_flock[curr_boid].y))
    {
        boid_flock[curr_boid].vy = boid_flock[curr_boid].vy + turnfactor;
    }
    else if (hitBottom(boid_flock[curr_boid].y))
    {
        boid_flock[curr_boid].vy = boid_flock[curr_boid].vy - turnfactor;
    }

    if (hitLeft(boid_flock[curr_boid].x))
    {
        boid_flock[curr_boid].vx = boid_flock[curr_boid].vx + turnfactor;
    }
    else if (hitRight(boid_flock[curr_boid].x))
    {
        boid_flock[curr_boid].vx = boid_flock[curr_boid].vx - turnfactor;
    }

    if (turn_on_predator)
    {
        // If there were any predators in the predatory range, turn away
        if (boid_flock[curr_boid].num_predators > 0)
        {
            if (boid_flock[curr_boid].predator_dy > 0)
            {
                boid_flock[curr_boid].vy = boid_flock[curr_boid].vy + predator_turnfactor;
            }
            if (boid_flock[curr_boid].predator_dy < 0)
            {
                boid_flock[curr_boid].vy = boid_flock[curr_boid].vy - predator_turnfactor;
            }
            if (boid_flock[curr_boid].predator_dx > 0)
            {
                boid_flock[curr_boid].vx = boid_flock[curr_boid].vx + predator_turnfactor;
            }
            if (boid_flock[curr_boid].predator_dx < 0)
            {
                boid_flock[curr_boid].vx = boid_flock[curr_boid].vx - predator_turnfactor;
            }
        }
    }
    

    fix15 speed;
    // Calculate the boid's speed
    // Calculated using the alpha beta max algorithm
    // speed = 1*v_max + 1/4 * v_min --> shift by 2 instead of multiply 0.25
    if (absfix15(boid_flock[curr_boid].vx) < absfix15(boid_flock[curr_boid].vy))
    {
        speed = absfix15(boid_flock[curr_boid].vy) + (absfix15(boid_flock[curr_boid].vx) >> 2);
    }
    else
    {
        speed = absfix15(boid_flock[curr_boid].vx) + (absfix15(boid_flock[curr_boid].vy) >> 2);
    }

    if (speed > maxspeed)
    {
        boid_flock[curr_boid].vx = boid_flock[curr_boid].vx - (boid_flock[curr_boid].vx >> 2);
        boid_flock[curr_boid].vy = boid_flock[curr_boid].vy - (boid_flock[curr_boid].vy >> 2);
    }
    if (speed < minspeed)
    {
        boid_flock[curr_boid].vx = boid_flock[curr_boid].vx + (boid_flock[curr_boid].vx >> 2);
        boid_flock[curr_boid].vy = boid_flock[curr_boid].vy + (boid_flock[curr_boid].vy >> 2);
    }

    // Update position using velocity
    boid_flock[curr_boid].x = boid_flock[curr_boid].x + boid_flock[curr_boid].vx;

    boid_flock[curr_boid].y = boid_flock[curr_boid].y + boid_flock[curr_boid].vy;

    if (boid_flock[curr_boid].x < int2fix15(0)) boid_flock[curr_boid].x = int2fix15(0);
    if (boid_flock[curr_boid].x > int2fix15(315)) boid_flock[curr_boid].x = int2fix15(315);
    if (boid_flock[curr_boid].y < int2fix15(0)) boid_flock[curr_boid].y = int2fix15(0);
    if (boid_flock[curr_boid].y > int2fix15(235)) boid_flock[curr_boid].y = int2fix15(235);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void predator_algo(uint8_t current_predator)
{
    // If the predator is near a box edge, make it turn by turnfactor
    if (hitTop(predators[current_predator].y))
    {
        predators[current_predator].vy = predators[current_predator].vy + turnfactor_predators;
    }
    if (hitBottom(predators[current_predator].y))
    {
        predators[current_predator].vy = predators[current_predator].vy - turnfactor_predators;
    }

    if (hitLeft(predators[current_predator].x))
    {
        predators[current_predator].vx = predators[current_predator].vx + turnfactor_predators;
    }
    if (hitRight(predators[current_predator].x))
    {
        predators[current_predator].vx = predators[current_predator].vx - turnfactor_predators;
    }

    fix15 speed;
    // Calculate the predator's speed
    if (absfix15(predators[current_predator].vx) < absfix15(predators[current_predator].vy))
    {
        speed = absfix15(predators[current_predator].vy) + (absfix15(predators[current_predator].vx) >> 2);
    }
    else
    {
        speed = absfix15(predators[current_predator].vx) + (absfix15(predators[current_predator].vy) >> 2);
    }

    if (speed > maxspeed_predators)
    {
        predators[current_predator].vx = predators[current_predator].vx - (predators[current_predator].vx >> 2);
        predators[current_predator].vy = predators[current_predator].vy - (predators[current_predator].vy >> 2);
    }
    if (speed < minspeed_predators)
    {
        predators[current_predator].vx = predators[current_predator].vx + (predators[current_predator].vx >> 2);
        predators[current_predator].vy = predators[current_predator].vy + (predators[current_predator].vy >> 2);
    }

    // Update position using velocity
    predators[current_predator].x = predators[current_predator].x + predators[current_predator].vx;
    predators[current_predator].y = predators[current_predator].y + predators[current_predator].vy;
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
            printf("turnfactor <float>\n\r");
            printf("visualrange <int>\n\r");
            printf("protectedrange <int>\n\r");
            printf("centeringfactor <float>\n\r");
            printf("avoidfactor <float>\n\r");
            printf("matchingfactor <float>\n\r");
            printf("numberBoids <int>\n\r");
            printf("numberPredators <int>\n\r");
            printf("maxspeed <int>\n\r");
            printf("minspeed <int>\n\r");
            printf("predatorFlockRange <int>\n\r");
            printf("predatorFlockTurnfactor <float>\n\r");
            printf("predatorRange <int>\n\r");
            printf("predatorTurnfactor <float>\n\r");
            printf("mood <int>\n\r");
            printf("splash\n\r");
            printf("from\n\n");
            printf("splashColor <float>\n\r");
        }
        else if (strcmp(cmd, "from") == 0)
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
                for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
                {
                    fillCircle(fix2int15(boid_flock[current_boid].x), fix2int15(boid_flock[current_boid].y), size_circle, BLACK);
                }

                for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
                {
                    fillCircle(fix2int15(predators[current_predator].x), fix2int15(predators[current_predator].y), size_circle, BLACK);
                }

                curr_N_boids = (uint16_t)(atoi(arg1));

                // Spawn boid flocks
                for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
                {
                    // All boid properties are zerod out from other thread
                    spawn(&boid_flock[current_boid].x, &boid_flock[current_boid].y, &boid_flock[current_boid].vx, &boid_flock[current_boid].vy);
                }

                for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
                {
                    spawn(&predators[current_predator].x, &predators[current_predator].y, &predators[current_predator].vx, &predators[current_predator].vy);
                    // Need to zero out alive counters
                    predators[current_predator].alive_counter = 0;
                }
            }
        }
        else if (strcmp(cmd, "numberPredators") == 0)
        {
            // erase predators and boids, and rerandomize initialization
            if (arg1 != NULL)
            {
                for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
                {
                    fillCircle(fix2int15(boid_flock[current_boid].x), fix2int15(boid_flock[current_boid].y), size_circle, BLACK);
                }

                for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
                {
                    fillCircle(fix2int15(predators[current_predator].x), fix2int15(predators[current_predator].y), size_circle, BLACK);
                }

                curr_N_predators = (uint16_t)(atoi(arg1));

                // Spawn boid flocks
                for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
                {
                    // All boid properties are zerod out from other thread
                    spawn(&boid_flock[current_boid].x, &boid_flock[current_boid].y, &boid_flock[current_boid].vx, &boid_flock[current_boid].vy);
                }

                // Spawn predators
                for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
                {
                    spawn(&predators[current_predator].x, &predators[current_predator].y, &predators[current_predator].vx, &predators[current_predator].vy);
                    // Need to zero out alive counters
                    predators[current_predator].alive_counter = 0;
                }
            }
        }
        else if (strcmp(cmd, "predatorFlockRange") == 0)
        {
            if (arg1 != NULL)
            {
                predator_flock_range = int2fix15(atoi(arg1));
            }
        }
        else if (strcmp(cmd, "predatorFlockTurnfactor") == 0)
        {
            if (arg1 != NULL)
            {
                predator_flock_turnfactor = float2fix15(atof(arg1));
            }
        }
        else if (strcmp(cmd, "predatorRange") == 0)
        {
            if (arg1 != NULL)
            {
                predator_range = int2fix15(atoi(arg1));
            }
        }
        else if (strcmp(cmd, "predatorTurnfactor") == 0)
        {
            if (arg1 != NULL)
            {
                predator_turnfactor = float2fix15(atof(arg1));
            }
        }
        else if (strcmp(cmd, "maxspeed") == 0)
        {
            if (arg1 != NULL)
            {
                maxspeed = int2fix15(atoi(arg1));
            }
        }
        else if (strcmp(cmd, "minspeed") == 0)
        {
            if (arg1 != NULL)
            {
                maxspeed = int2fix15(atoi(arg1));
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
                if (predator_spawn_index == curr_N_predators)
                {
                    predator_spawn_index = 0;
                }
            }
        }
        else if (strcmp(cmd, "splashColor") == 0)
        {
            if (arg1 != NULL)
            {
                Splash_Color = atof(arg1);
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

    // Variables for maintaining frame rate
    int begin_time;
    int spare_time;
    char color_to_draw = hsv2rgb(120, 0.75, 1);

    // Spawn boid flocks
    for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
    {
        spawn(&boid_flock[current_boid].x, &boid_flock[current_boid].y, &boid_flock[current_boid].vx, &boid_flock[current_boid].vy);
        boid_flock[current_boid].hue = 180;
    }

    // Spawn all predators
    for (uint8_t current_predator = 0; current_predator < curr_N_predators; current_predator++)
    {
        spawn(&predators[current_predator].x, &predators[current_predator].y, &predators[current_predator].vx, &predators[current_predator].vy);
    }

    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();

        // If a new note is detected, change the necessary predators to their color
        if (turn_on_predator)
        {
            if (curr_N_predators == 1)
            {
                predators[0].hue = animate_mood_1;
            }
            else if (curr_N_predators == 2)
            {
                predators[0].hue = animate_mood_1;
                predators[1].hue = animate_mood_2;
            }
            else if (curr_N_predators == 3)
            {
                predators[0].hue = animate_mood_1;
                predators[1].hue = animate_mood_2;
                predators[2].hue = animate_mood_3;
            }
        }

        // Every ten times through the while loop, calculate the overall mood given the past 20 notes
        if (counter_hue > 10)
        {
            change_to_overall = true;
            counter_hue = 0;
        }
        else counter_hue++;

        for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
        {
            
            // update boid's position and velocity 
            // (and color if turn_on_predator and/or change_to_overall are true)
            boid_algo_init_calc(current_boid);
        }

        if (change_to_overall) change_to_overall = false;

        // Move predators
        for (uint8_t current_predator = 0; current_predator < 3; current_predator++)
        {
            predator_algo(current_predator);
        }

        for (uint16_t current_boid = 0; current_boid < curr_N_boids; current_boid++)
        {

            // Update boid state
            boid_algo_update(current_boid);

            // Find row and column of the tile the boid is in
            int col = (int)(fix2int15(boid_flock[current_boid].x) / tile_side);
            int row = (int)(fix2int15(boid_flock[current_boid].y) / tile_side);

            int tile_index = row * width + col;

            // set the color of the tile to the color of the boids in the tile
            // value will be averaged later
            tiles[tile_index].total_hue += boid_flock[current_boid].hue;
            tiles[tile_index].num_boids++;

            // Set all values needed for boid calculate back to 0
            boid_flock[current_boid].close_dx = 0;
            boid_flock[current_boid].close_dy = 0;
            boid_flock[current_boid].xpos_avg = 0;
            boid_flock[current_boid].ypos_avg = 0;
            boid_flock[current_boid].xvel_avg = 0;
            boid_flock[current_boid].yvel_avg = 0;
            boid_flock[current_boid].neighboring_boids = 0;

            boid_flock[current_boid].predator_flock_dx = 0;
            boid_flock[current_boid].predator_flock_dy = 0;
            boid_flock[current_boid].num_flock_predators = 0;

            boid_flock[current_boid].predator_dx = 0;
            boid_flock[current_boid].predator_dy = 0;
            boid_flock[current_boid].num_predators = 0;
        }
        
        for (int i = 0; i < total_tiles; i++)
        {
            
            int num_boids = tiles[i].num_boids;

            if (num_boids > 0)
            {
                // calculates average color of boids within tile
                int avg_hue = tiles[i].total_hue / tiles[i].num_boids;
                color_to_draw = hsv2rgb(avg_hue, 0.75, 1);
            }
            else
            {
                // if there are no boids in the tile, sset it to the overall mood
                color_to_draw = hsv2rgb(overall_mood, 0.75, 1);
            }

            // draw the current tile
            fillRect(tiles[i].x, tiles[i].y, tile_side, tile_side, color_to_draw);

            tiles[i].total_hue = 0;
            tiles[i].num_boids = 0;

        }

        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        PT_YIELD_usec(100);

        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// fft on core 1
static PT_THREAD(protothread_FFT(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    ///////////////////////////////////////////////////////////////////////////
    // For Music

    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan));
    // Start the ADC
    adc_run(true);

    // Declare some static variables
    static int height;         // for scaling display
    static float max_freqency; // holds max frequency
    static int i;              // incrementing loop variable

    // static fix15 max_fr;   // temporary variable for max freq calculation
    static int max_fr_dex; // index of max frequency

    ///////////////////////////////////////////////////////////////////////////

    while (1)
    {
        PT_YIELD_usec(1000);

        // Wait for NUM_SAMPLES samples to be gathered
        // Measure wait time with timer. THIS IS BLOCKING
        dma_channel_wait_for_finish_blocking(sample_chan);

        // Copy/window elements into a fixed-point array
        for (i = 0; i < NUM_SAMPLES; i++)
        {
            fr[i] = multfix15(int2fix15((int)sample_array[i]), window[i]);
            fi[i] = (fix15)0;
        }

        // Zero max frequency and max frequency index
        max_fr = 0;
        max_fr_dex = 0;

        // Restart the sample channel, now that we have our copy of the samples
        dma_channel_start(control_chan);

        // Compute the FFT
        FFTfix(fr, fi);

        // Find the magnitudes (alpha max plus beta min)
        for (int i = 0; i < (NUM_SAMPLES >> 1); i++)
        {
            // get the approx magnitude
            fr[i] = abs(fr[i]);
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = max(fr[i], fi[i]) +
                    multfix15(min(fr[i], fi[i]), zero_point_4);

            // Keep track of top 3 frequencies
            if (fr[i] > max_fr && i > 2)
            {
                max_fr = fr[i];
                max_fr_dex = i;
                current_loudest_3_notes[2].mag = current_loudest_3_notes[1].mag;
                current_loudest_3_notes[1].mag = current_loudest_3_notes[0].mag;
                current_loudest_3_notes[0].mag = max_fr;
                current_loudest_3_notes[2].freq = current_loudest_3_notes[1].freq;
                current_loudest_3_notes[1].freq = current_loudest_3_notes[0].freq;
                current_loudest_3_notes[0].freq = int2fix15(i);
            }
            // Only tracks the top frequency for the first 3 samples
            else if(fr[i] > max_fr)
            {
                max_fr = fr[i];
                max_fr_dex = i;
            }
        }
        // Calculates the difference in the current highest amplitude between the previous note and the new note
        percent_diff = divfix(current_loudest_3_notes[0].mag - old_note_mag, old_note_mag);

        if (current_loudest_3_notes[0].mag < mag_threshold)
        {
            // if the new note was too quiet, no new note was played
            turn_on_predator = false;
            curr_N_predators = 0;
        }
        else if (abs(percent_diff) > percent_diff_threshold)
        {  
            // if the difference between the previous note and new note is greater than 1%, then a new note was played
            old_note_mag = current_loudest_3_notes[0].mag;
            // Calculate frequency
            current_loudest_3_notes[0].freq = multfix15(current_loudest_3_notes[0].freq, freq_calc);
            current_loudest_3_notes[1].freq = multfix15(current_loudest_3_notes[1].freq, freq_calc);
            current_loudest_3_notes[2].freq = multfix15(current_loudest_3_notes[2].freq, freq_calc);
            curr_N_predators = music_stuff();
            turn_on_predator = true;  
        }

        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // fft thread


// Core 1 entry point (main() for core 1)
void core1_entry()
{

    /////////////////////////////////////////////////////////////////////////////
    // Add and schedule threads
    pt_add_thread(protothread_FFT);
    
    pt_schedule_start;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
    // Overclocking
    // set_sys_clock_khz(250000, true);

    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

    /////////////////////////////////////////////////////////////////////////////
    // For Music

    ///////////////////////////////////////////////////////////////////////////////
    // ============================== ADC CONFIGURATION ==========================
    //////////////////////////////////////////////////////////////////////////////
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC_PIN);

    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init();

    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    adc_select_input(ADC_CHAN);

    // Setup the FIFO
    adc_fifo_setup(
        true,  // Write each completed conversion to the sample FIFO
        true,  // Enable DMA data request (DREQ)
        1,     // DREQ (and IRQ) asserted when at least 1 sample present
        false, // We won't see the ERR bit because of 8 bit reads; disable.
        true   // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock. This is setup
    // to grab a sample at 10kHz (48Mhz/10kHz - 1)
    adc_set_clkdiv(ADCCLK / Fs);

    // Populate the sine table and Hann window table
    int ii;
    for (ii = 0; ii < NUM_SAMPLES; ii++)
    {
        Sinewave[ii] = float2fix15(sin(6.283 * ((float)ii) / (float)NUM_SAMPLES));
        window[ii] = float2fix15(0.5 * (1.0 - cos(6.283 * ((float)ii) / ((float)NUM_SAMPLES))));
    }

    /////////////////////////////////////////////////////////////////////////////////
    // ============================== ADC DMA CONFIGURATION =========================
    /////////////////////////////////////////////////////////////////////////////////

    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);

    // ADC SAMPLE CHANNEL
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
                          &c2,           // channel config
                          sample_array,  // dst
                          &adc_hw->fifo, // src
                          NUM_SAMPLES,   // transfer count
                          false          // don't start immediately
    );

    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32); // 32-bit txfers
    channel_config_set_read_increment(&c3, false);           // no read incrementing
    channel_config_set_write_increment(&c3, false);          // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);           // chain to sample chan

    dma_channel_configure(
        control_chan,                        // Channel to be configured
        &c3,                                 // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr, // Write address (channel 0 read address)
        &sample_address_pointer,             // Read address (POINTER TO AN ADDRESS)
        1,                                   // Number of transfers, in this case each is 4 byte
        false                                // Don't start immediately.
    );

    ////////////////////////////////////////////////////////////////////////////////

    for (int i = 0; i < total_tiles; i++)
    {
        // Do color processing
        int row = i / width;
        int col = i - row * width;
        tiles[i].x = col * tile_side;
        tiles[i].y = row * tile_side;
    }


    // Launch core 1
    multicore_reset_core1();
    multicore_launch_core1(&core1_entry);

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);
    // pt_add_thread(protothread_toggle25);

    // start scheduler
    pt_schedule_start;
}

/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration calculates an FFT of audio input, and
 * then displays that FFT on a 640x480 VGA display.
 * 
 * Core 0 computes and displays the FFT. Core 1 blinks the LED.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 26 ---> Audio input [0-3.3V]
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - ADC channel 0
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include VGA graphics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// Define the LED pin
#define LED     25

// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

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

// DMA channels for sampling ADC (VGA driver uses 0 and 1)
int sample_chan = 2 ;
int control_chan = 3 ;

// Max and min macros
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

// 0.4 in fixed point (used for alpha max plus beta min)
fix15 zero_point_4 = float2fix15(0.4) ;

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES] ;
// And here's where we'll copy those samples for FFT calculation
fix15 fr[NUM_SAMPLES] ;
fix15 fi[NUM_SAMPLES] ;

// Sine table for the FFT calculation
fix15 Sinewave[NUM_SAMPLES]; 
// Hann window table for FFT calculation
fix15 window[NUM_SAMPLES]; 

// Pointer to address of start of sample buffer
uint8_t * sample_address_pointer = &sample_array[0] ;


/////////////////////////// Music Identification Code ////////////////////////////////

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

struct note_mag_freq_array current_loudest_3_notes[3] ;// Take top 3 notes of current sample mag and freq
struct note_mag_freq_mood_array past_10_notes[10] ; // Store past 10 high notes

// Global variables
volatile float animate_mood ; // Range from 0-2; 0 == major, 1 == minor, 2 == dissonant
volatile float overall_mood ;
bool calculate_new_note = false;
fix15 percent_diff = 0;
fix15 percent_diff_threshold = float2fix15(0.01) ;
fix15 old_note_mag = float2fix15(0.001);
fix15 freq_calc = float2fix15(Fs/NUM_SAMPLES) ;
fix15 percentage_high_note_diff = float2fix15(0.25) ;
fix15 mag_threshold = float2fix15(0.5);


///////////////////////////////////////////////////////////////////////////////////
///////////////////// Cents to intervals and moods functions //////////////////////

int solve_for_cents(fix15 a, fix15 b) {
    float freq_ratio ;
    float log2 ;
    int cents ;
    freq_ratio = fix2float15(divfix(b, a));
    log2 = log10(freq_ratio) / log10(2);
    cents = (int)(12 * log2);

    return cents;
}

float identify_music_mood(int cents) {
    float mood ;
    while (cents > 12)
    {
        cents -= 12; 
    }
    
    if (cents == 0 || cents == 4 || cents == 5 || cents == 7) 
    {
        mood = 0; // 0 = major, 1 = minor, 2 = dissonant
    }
    else if (cents == 3 || cents == 2 || cents == 8 || cents == 9 || cents == 12)
    {
        mood = 1;
    }
    else if (cents == 1 || cents == 6 || cents == 10 || cents == 11)
    {
        mood = 2;
    }
    return mood;
    
}

void music_stuff() {

    fix15 percentage_high_note_2 ;
    fix15 percentage_high_note_3 ;
    int cents_with_prev_note ;
    fix15 top_note = 0;
    fix15 middle_note = 0;
    fix15 bottom_note = 0;
    float curr_mood ;
    float interval ;
    float interval_low ;
    float interval_high ;
    float animate_mood_1 ;
    float animate_mood_2 ;
    float sum_mood = 0;

    percentage_high_note_2 = divfix(current_loudest_3_notes[1].mag - current_loudest_3_notes[0].mag, current_loudest_3_notes[0].mag);
    percentage_high_note_3 = divfix(current_loudest_3_notes[2].mag - current_loudest_3_notes[0].mag, current_loudest_3_notes[0].mag);

    if (abs(percentage_high_note_2) > percentage_high_note_diff && abs(percentage_high_note_3) > percentage_high_note_diff) {
        // Only loudest note
        top_note = current_loudest_3_notes[0].freq;
        cents_with_prev_note = solve_for_cents(past_10_notes[9].freq, top_note);
        curr_mood = identify_music_mood(cents_with_prev_note);
        animate_mood = curr_mood;
    }
    else if (abs(percentage_high_note_2) > percentage_high_note_diff){
        // Only notes 0 and 2
        if (current_loudest_3_notes[0].freq > current_loudest_3_notes[2].freq)
        {
            top_note = current_loudest_3_notes[0].freq;
            bottom_note = current_loudest_3_notes[2].freq;
        }
        else 
        {
            top_note = current_loudest_3_notes[2].freq;
            bottom_note = current_loudest_3_notes[0].freq; 
        }
        interval = solve_for_cents(bottom_note, top_note);
        animate_mood = identify_music_mood(interval);
    }
    else if (abs(percentage_high_note_3) > percentage_high_note_diff){
        // Only notes 0 and 1
        if (current_loudest_3_notes[0].freq > current_loudest_3_notes[1].freq)
        {
            top_note = current_loudest_3_notes[0].freq;
            bottom_note = current_loudest_3_notes[1].freq;
        }
        else 
        {
            top_note = current_loudest_3_notes[1].freq;
            bottom_note = current_loudest_3_notes[0].freq; 
        }
        interval = solve_for_cents(bottom_note, top_note);
        animate_mood = identify_music_mood(interval);
    }
    else {
        // All 3 notes
        for (int m=0; m<3; m++) {
            if (current_loudest_3_notes[m].freq > top_note) top_note = current_loudest_3_notes[m].freq;
            else if (current_loudest_3_notes[m].freq > middle_note) middle_note = current_loudest_3_notes[m].freq;
            else bottom_note = current_loudest_3_notes[m].freq;
        }
        interval_low = solve_for_cents(bottom_note, middle_note);
        interval_high = solve_for_cents(middle_note, top_note);
        animate_mood_1 = identify_music_mood(interval_low);
        animate_mood_2 = identify_music_mood(interval_high);
        animate_mood = (animate_mood_1 + animate_mood_2)/2;
    } 
    
    if (bottom_note != 0 && middle_note != 0);
    else {
        cents_with_prev_note = solve_for_cents(past_10_notes[9].freq, top_note);
        curr_mood = identify_music_mood(cents_with_prev_note);
    }

    for (int i = 0; i < 10; i++) {
        if (i == 9)
        {
             past_10_notes[i].freq = top_note;
             past_10_notes[i].mood = curr_mood;
             sum_mood += past_10_notes[i].mood;
        }
        else 
        {
            past_10_notes[i].freq = past_10_notes[i+1].freq;
            past_10_notes[i].mood = past_10_notes[i+1].mood;
            sum_mood += past_10_notes[i].mood;
        }
    }
    printf("sum_mood = %f\n",sum_mood);
    overall_mood = (float)(sum_mood/10);
    
}


/////////////////////////// END Music Identification Code ////////////////////////////////


// Peforms an in-place FFT. For more information about how this
// algorithm works, please see https://vanhunteradams.com/FFT/FFT.html
void FFTfix(fix15 fr[], fix15 fi[]) {
    
    unsigned short m;   // one of the indices being swapped
    unsigned short mr ; // the other index being swapped (r for reversed)
    fix15 tr, ti ; // for temporary storage while swapping, and during iteration
    
    int i, j ; // indices being combined in Danielson-Lanczos part of the algorithm
    int L ;    // length of the FFT's being combined
    int k ;    // used for looking up trig values from sine table
    
    int istep ; // length of the FFT which results from combining two FFT's
    
    fix15 wr, wi ; // trigonometric values from lookup table
    fix15 qr, qi ; // temporary variables used during DL part of the algorithm
    
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here: 
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    for (m=1; m<NUM_SAMPLES_M_1; m++) {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT ;
        // don't swap that which has already been swapped
        if (mr<=m) continue ;
        // swap the bit-reveresed indices
        tr = fr[m] ;
        fr[m] = fr[mr] ;
        fr[mr] = tr ;
        ti = fi[m] ;
        fi[m] = fi[mr] ;
        fi[mr] = ti ;
    }
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1 ;
    // Log2 of number of samples, minus 1
    k = LOG2_NUM_SAMPLES - 1 ;
    // While the length of the FFT's being combined is less than the number 
    // of gathered samples . . .
    while (L < NUM_SAMPLES) {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L<<1 ;
        // For each element in the FFT's that are being combined . . .
        for (m=0; m<L; ++m) { 
            // Lookup the trig values for that element
            j = m << k ;                         // index of the sine table
            wr =  Sinewave[j + NUM_SAMPLES/4] ; // cos(2pi m/N)
            wi = -Sinewave[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i=m; i<NUM_SAMPLES; i+=istep) {
                // j gets the index of the FFT element being combined with i
                j = i + L ;
                // compute the trig terms (bottom half of the above matrix)
                tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]) ;
                ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                qr = fr[i]>>1 ;
                qi = fi[i]>>1 ;
                // compute the new values at each index
                fr[j] = qr - tr ;
                fi[j] = qi - ti ;
                fr[i] = qr + tr ;
                fi[i] = qi + ti ;
            }    
        }
        --k ;
        L = istep ;
    }
}

// Runs on core 0
static PT_THREAD (protothread_fft(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt) ;
    printf("Starting capture\n") ;
    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan)) ;
    // Start the ADC
    adc_run(true) ;

    // Declare some static variables
    static int height ;             // for scaling display
    static float max_freqency ;     // holds max frequency
    static int i ;                  // incrementing loop variable

    static fix15 max_fr ;           // temporary variable for max freq calculation
    static int max_fr_dex ;         // index of max frequency

    // Write some text to VGA
    setTextColor(WHITE) ;
    // setCursor(65, 0) ;
    // setTextSize(1) ;
    // writeString("Raspberry Pi Pico") ;
    // setCursor(65, 10) ;
    // writeString("FFT demo") ;
    // setCursor(65, 20) ;
    // writeString("Hunter Adams") ;
    // setCursor(65, 30) ;
    // writeString("vha3@cornell.edu") ;
    setCursor(250, 0) ;
    setTextSize(1) ;
    writeString("Max freqency:") ;
    setCursor(250, 10) ;
    setTextSize(1) ;
    writeString("Magnitude:") ;
    setCursor(250, 20) ;
    setTextSize(1) ;
    writeString("Animate Mood:") ;
    setCursor(250, 30) ;
    setTextSize(1) ;
    writeString("Overall Mood:") ;

    // Will be used to write dynamic text to screen
    static char freqtext[40];


    while(1) {
        // Wait for NUM_SAMPLES samples to be gathered
        // Measure wait time with timer. THIS IS BLOCKING
        dma_channel_wait_for_finish_blocking(sample_chan);

        // Copy/window elements into a fixed-point array
        for (i=0; i<NUM_SAMPLES; i++) {
            fr[i] = multfix15(int2fix15((int)sample_array[i]), window[i]) ;
            fi[i] = (fix15) 0 ;
        }

        // Zero max frequency and max frequency index
        max_fr = 0 ;
        max_fr_dex = 0 ;

        // Restart the sample channel, now that we have our copy of the samples
        dma_channel_start(control_chan) ;

        // Compute the FFT
        FFTfix(fr, fi) ;

        // Find the magnitudes (alpha max plus beta min)
        for (int i = 0; i < (NUM_SAMPLES>>1); i++) {  
            // get the approx magnitude
            fr[i] = abs(fr[i]); 
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = max(fr[i], fi[i]) + 
                    multfix15(min(fr[i], fi[i]), zero_point_4); 
            // Keep track of top 3
            if(fr[i] > max_fr && i>4) {
                max_fr = fr[i] ;
                // printf("mag = %d\n",fix2int15(max_fr));
                max_fr_dex = i ;
                current_loudest_3_notes[2].mag = current_loudest_3_notes[1].mag;
                current_loudest_3_notes[1].mag = current_loudest_3_notes[0].mag;
                current_loudest_3_notes[0].mag = max_fr;
                current_loudest_3_notes[2].freq = current_loudest_3_notes[1].freq;
                current_loudest_3_notes[1].freq = current_loudest_3_notes[0].freq;
                current_loudest_3_notes[0].freq = int2fix15(i);
                percent_diff = divfix(max_fr - old_note_mag,old_note_mag); 
                if (abs(percent_diff) > percent_diff_threshold && current_loudest_3_notes[0].mag > mag_threshold) {
                    calculate_new_note = true;
                    old_note_mag = current_loudest_3_notes[0].mag;
                }
                
            }
        }
        if (calculate_new_note)
        {
            current_loudest_3_notes[0].freq = multfix15(current_loudest_3_notes[0].freq,freq_calc);
            current_loudest_3_notes[1].freq = multfix15(current_loudest_3_notes[1].freq,freq_calc);
            current_loudest_3_notes[2].freq = multfix15(current_loudest_3_notes[2].freq,freq_calc) ;
            calculate_new_note = false;
            music_stuff();
        }
        
        // Compute max frequency in Hz
        // max_freqency = max_fr_dex * (Fs/NUM_SAMPLES) ;
        // // printf("mag = %d\n",fix2int15(max_fr));
        // printf("frequency = %1.2f\n", max_freqency);
        // printf("freq_calc = %d\n", (Fs/NUM_SAMPLES));


        // Display on VGA
        fillRect(350, 0, 400, 40, BLACK); // red box
        sprintf(freqtext, "%d", fix2int15(current_loudest_3_notes[0].freq)) ;
        setCursor(350, 0) ;
        setTextSize(1) ;
        writeString(freqtext) ;
        sprintf(freqtext, "%1.3f", fix2float15(current_loudest_3_notes[2].mag)) ;
        setCursor(350, 10) ;
        setTextSize(1) ;  
        writeString(freqtext) ;
        sprintf(freqtext, "%1.2f", animate_mood) ;
        setCursor(350, 20) ;
        setTextSize(1) ;
        writeString(freqtext) ;
        sprintf(freqtext, "%1.2f", overall_mood) ;
        setCursor(350, 30) ;
        setTextSize(1) ;
        writeString(freqtext) ;

        // Update the FFT display
        for (int i=5; i<(NUM_SAMPLES>>1); i++) {
            drawVLine(59+i, 50, 429, BLACK);
            height = fix2int15(multfix15(fr[i], int2fix15(36))) ;
            drawVLine(59+i, 479-height, height, WHITE);
        }

    }
    PT_END(pt) ;
}

static PT_THREAD (protothread_blink(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt) ;
    while (1) {
        // Toggle LED, then wait half a second
        gpio_put(LED, !gpio_get(LED)) ;
        PT_YIELD_usec(500000) ;
    }
    PT_END(pt) ;
}

// Core 1 entry point (main() for core 1)
void core1_entry() {
    // Add and schedule threads
    pt_add_thread(protothread_blink) ;
    pt_schedule_start ;
}

// Core 0 entry point
int main() {
    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA() ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ///////////////////////////////////////////////////////////////////////////////
    // ============================== ADC CONFIGURATION ==========================
    //////////////////////////////////////////////////////////////////////////////
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC_PIN);

    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init() ;

    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    adc_select_input(ADC_CHAN) ;

    // Setup the FIFO
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock. This is setup
    // to grab a sample at 10kHz (48Mhz/10kHz - 1)
    adc_set_clkdiv(ADCCLK/Fs);


    // Populate the sine table and Hann window table
    int ii;
    for (ii = 0; ii < NUM_SAMPLES; ii++) {
        Sinewave[ii] = float2fix15(sin(6.283 * ((float) ii) / (float)NUM_SAMPLES));
        window[ii] = float2fix15(0.5 * (1.0 - cos(6.283 * ((float) ii) / ((float)NUM_SAMPLES))));
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
        &c2,            // channel config
        sample_array,   // dst
        &adc_hw->fifo,  // src
        NUM_SAMPLES,    // transfer count
        false            // don't start immediately
    );

    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);      // 32-bit txfers
    channel_config_set_read_increment(&c3, false);                // no read incrementing
    channel_config_set_write_increment(&c3, false);               // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);                // chain to sample chan

    dma_channel_configure(
        control_chan,                         // Channel to be configured
        &c3,                                // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr,  // Write address (channel 0 read address)
        &sample_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers, in this case each is 4 byte
        false                               // Don't start immediately.
    );

    // Launch core 1
    multicore_launch_core1(core1_entry);

    // Add and schedule core 0 threads
    pt_add_thread(protothread_fft) ;
    pt_schedule_start ;

}

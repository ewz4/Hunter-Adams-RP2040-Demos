/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *  - GPIO 5 ---> PWM output
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV 25.0
uint slice_num;

// PWM duty cycle
volatile int control = 0;
volatile int old_control = 0;
volatile int motor_disp = 0;

// Angle Variables
fix15 zeroopt001 = float2fix15(0.01);
fix15 zeroopt999 = float2fix15(0.99);
fix15 filtered_ay = 0;
fix15 filtered_az = 0;
fix15 accel_angle = 0;
fix15 gyro_angle_delta = 0;
fix15 complementary_angle = 0;
fix15 time_gyro = float2fix15(0.001);
fix15 adjust_angle = int2fix15(90);

volatile int counter_0 = 0;

// Controller Parameters
fix15 angle_reference = 0;
fix15 error ;
fix15 last_error = 0;
fix15 proportional ;
fix15 integral ;
fix15 derivative ;
fix15 integral_wind_up = 2000;
fix15 kp = 0;
fix15 ki = 0;
fix15 kd = 0;
volatile int controller = 0;

// Interrupt service routine
void on_pwm_wrap()
{

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    filtered_ay = filtered_ay + ((acceleration[1] - filtered_ay) >> 6);
    filtered_az = filtered_az + ((acceleration[2] - filtered_az) >> 6);
    accel_angle = multfix15(float2fix15(atan2(filtered_ay, filtered_az) + 1.5708), float2fix15(180 / M_PI));
    gyro_angle_delta = multfix15(gyro[0], time_gyro);
    complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeroopt999) + multfix15(accel_angle, zeroopt001);

    // Controller Calculation
    
    if (controller = 0)
    {
        // Do nothing
    }
    else
    {
        last_error = error ;
        error = angle_reference - complementary_angle ;

        // Calculate controller values
        poportional = multfix15(kp, error);

        integral += error;
        // Avoid integral wind up
        if (sign(integral) != sign(integral))
        {
            integral = 0;
        }
        else if (integral > integral_wind_up)
        {
            integral = integral_wind_up;
        }
        else if (integral < -integral_wind_up)
        {
            integral = -integral_wind_up;
        }

        derviative = multfix15(kd, (error - last_error));

        else if (controller = 1)
        {
            // P controller
            control = fix2int15(proportional);
        }
        else if (controller = 2)
        {
            // PI controller
            control = fix2int15(proportional + multfix15(integral,ki))
        }
        else if (controller = 3)
        {
            // PD controller
            control = fix2int15(proportional + derivative)
        }
        else if (controller = 4)
        {
            // PID controller
            control = fix2int15(proportional + derivative + multfix15(integral,ki))
        }
        if (control > 5000) control = 5000;
        else if (control < 0) control = 0;
    }
    

    // Update duty cycle
    if (control != old_control)
    {
        old_control = control;
        pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
    }

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD(protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt);

    // We will start drawing at column 81
    static int xcoord = 81;

    // Rescale the measurements for display
    static float OldRange = 500.; // (+/- 250)
    static float NewRange = 150.; // (looks nice on VGA)
    static float OldMin = -250.;
    static float OldMax = 250.;

    // Control rate of drawing
    static int throttle;

    // Draw the static aspects of the display
    setTextSize(1);
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN);
    drawHLine(75, 355, 5, CYAN);
    drawHLine(75, 280, 5, CYAN);
    drawVLine(80, 280, 150, CYAN);
    sprintf(screentext, "90");
    setCursor(50, 350);
    writeString(screentext);
    sprintf(screentext, "180");
    setCursor(50, 280);
    writeString(screentext);
    sprintf(screentext, "0");
    setCursor(50, 425);
    writeString(screentext);

    // Draw top plot
    drawHLine(75, 230, 5, CYAN);
    drawHLine(75, 155, 5, CYAN);
    drawHLine(75, 80, 5, CYAN);
    drawVLine(80, 80, 150, CYAN);
    sprintf(screentext, "2500");
    setCursor(50, 150);
    writeString(screentext);
    sprintf(screentext, "5000");
    setCursor(45, 75);
    writeString(screentext);
    sprintf(screentext, "0");
    setCursor(45, 225);
    writeString(screentext);

    while (true)
    {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold)
        {
            // Zero drawspeed controller
            throttle = 0;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK);

            ///////////////// FROM DEMO ////////////////////////////////////////
            // // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[0])*120.0)-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[2])*120.0)-OldMin)/OldRange)), GREEN) ;

            // // Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;

            // Calculate Angle
            // zeroopt001 = float2fix15(0.01);
            // zeroopt999 = float2fix15(0.99);

            // Plot Angle
            drawPixel(xcoord, 430 - (int)((float)(fix2int15(accel_angle)) * 150 / 180), WHITE);
            drawPixel(xcoord, 430 - (int)((float)(fix2int15(gyro_angle_delta)) * 150 / 180), RED);
            drawPixel(xcoord, 430 - (int)((float)(fix2int15(complementary_angle)) * 150 / 180), GREEN);

            // Plot Duty Cycle
            drawPixel(xcoord, 230 - (int)((float)(control)*150 / 5000), GREEN);

            if (counter_0 > 30)
            {
                fillRect(500, 25, 600, 70, BLACK);
                sprintf(screentext, "Comp_angle=%d", fix2int15(complementary_angle));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 50);
                writeString(screentext);
                sprintf(screentext, "Duty Cycle=%d", control);
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 25);
                writeString(screentext);
                counter_0 = 0;
            }
            counter_0++;
            // Update horizontal cursor
            if (xcoord < 609)
            {
                xcoord += 1;
            }
            else
            {
                xcoord = 81;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // From example
    // static char classifier;
    // static int test_in;
    // static float float_in;

    // My own
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
        printf("timestep\n\r");
        printf("duty cycle -- (0-5000 only)\n\r");
        printf('desired angle')
        printf("set control none\n\r");
        printf("set control P\n\r");
        printf("set control PI\n\r");
        printf("set control PD\n\r");
        printf("set control PID\n\r");
        printf("kp\n\r");
        printf("ki\n\r");
        printf("kd\n\r");
        printf("integral wind up\n\r")
        }
        else if (strcmp(cmd, "timestep"))
        {
            if (arg1 != NULL)
            {
                threshold = (int)(atof(arg1));
            }
        }
        else if (strcmp(cmd, "duty cycle"))
        {
            if (arg1 != NULL)
            {
                test_in = (int)(atof(arg1));
                if (test_in > 5000) continue;
                else if (test_in < 0) continue;
                else control = test_in;
            }
        }
        else if (strcmp(cmd, "desried angle"))
        {
            if (arg1 != NULL)
            {
                angle_reference = float2fix15(atof(arg1));
            }
        }
        else if (strcmp(cmd, "set control") == 0)
        {
            if (strcmp(arg1, "none") == 0)
            {
                controller = 0;
            }
            if (strcmp(arg1, "P") == 0)
            {
                controller = 1;
            }
            else if (strcmp(arg1, "PI") == 0)
            {
                controller = 2;
            }
            else if (strcmp(arg1, "PD") == 0)
            {
                controller = 3;
            }
            else if (strcmp(arg1, "PID") == 0)
            {
                controller = 4;
            }
        }
        else if (strcmp(cmd, "kp") == 0)
        {
            if (arg1 != NULL)
            {
                kp = float2fix15(atof(arg1));
            }
        }
        else if (strcmp(cmd, "ki") == 0)
        {
            if (arg1 != NULL)
            {
                ki = float2fix15(atof(arg1));
            }
        }
        else if (strcmp(cmd, "kd") == 0)
        {
            if (arg1 != NULL)
            {
                kd = float2fix15(atof(arg1));
            }
        } 
        else if (strcmp(cmd, "integral wind up") == 0)
        {
            if (arg1 != NULL)
            {
                integral_wind_up = float2fix15(atof(arg1));
            }
        }
    }
    // {
    //     sprintf(pt_serial_out_buffer, "input a command: ");
    //     serial_write;
    //     // spawn a thread to do the non-blocking serial read
    //     serial_read;
    //     // convert input string to number
    //     sscanf(pt_serial_in_buffer, "%c", &classifier);

    //     // num_independents = test_in ;
    //     if (classifier == 't')
    //     {
    //         sprintf(pt_serial_out_buffer, "timestep: ");
    //         serial_write;
    //         serial_read;
    //         // convert input string to number
    //         sscanf(pt_serial_in_buffer, "%d", &test_in);
    //         if (test_in > 0)
    //         {
    //             threshold = test_in;
    //         }
    //     }
    //     if (classifier == 'd')
    //     {
    //         sprintf(pt_serial_out_buffer, "input a duty cycle (0-5000): ");
    //         serial_write;
    //         serial_read;
    //         // convert input string to number
    //         sscanf(pt_serial_in_buffer, "%d", &test_in);
    //         if (test_in > 5000)
    //             continue;
    //         else if (test_in < 0)
    //             continue;
    //         else
    //             control = test_in;
    //     }
    // }
    PT_END(pt);
}

// Entry point for core 1
void core1_entry()
{
    pt_add_thread(protothread_vga);
    pt_schedule_start;
}

int main()
{

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}

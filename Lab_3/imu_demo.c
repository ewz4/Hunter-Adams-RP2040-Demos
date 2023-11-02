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
 *  - GPIO 7 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *  - GPIO 5 ---> PWM output
 *  - GPIO 10 ---> Button
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
#include "hardware/sync.h"
#include "hardware/spi.h"
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
fix15 old_integral ;
fix15 integral ;
fix15 integral_part ;
fix15 integral_proportion = float2fix15(0.98);
fix15 derivative ;
fix15 integral_wind_up = int2fix15(16000); // 15.16 one 15 is a sign flip
fix15 kp = int2fix15(150);
fix15 ki = float2fix15(0.3);
fix15 kd = int2fix15(16000);
volatile int controller = 4;
fix15 error_array[5] = {0,0,0,0,0};

// Button
#define BUTTON 10
bool pressed = false;
uint32_t t1 = 0;
uint32_t t2 = 0;
uint32_t t3 = 0;

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
    
    if (controller == 0)
    {
        // Do nothing
    }
    else
    {
        // last_error = error ;
        error = angle_reference - complementary_angle ;

        error_array[4] = error_array[3];
        error_array[3] = error_array[2];
        error_array[2] = error_array[1];
        error_array[1] = error_array[0];
        error_array[0] = error;
        

        // Calculate controller values
        proportional = multfix15(kp, error);

        // old_integral = integral;
        integral += error;

        if (integral > integral_wind_up)
        {
            integral = integral_wind_up; 
            //integral = divfix(integral_part,ki);
        }
        else if (integral < -integral_wind_up)
        {
            integral = -integral_wind_up;
            // integral = divfix(integral_part,ki);
        }
        
        // // Avoid integral wind up
        // if ((error < 0) != (last_error < 0))
        // {
        //     // integral = multfix15(integral, integral_proportion);
        //     integral -= error;
        // }

        integral_part = multfix15(integral,ki);
        
        // if (integral_part > integral_wind_up)
        // {
        //     integral_part = integral_wind_up; 
        //     //integral = divfix(integral_part,ki);
        // }
        // else if (integral_part < -integral_wind_up)
        // {
        //     integral_part = -integral_wind_up;
        //     // integral = divfix(integral_part,ki);
        // }

        derivative = multfix15(kd, (error_array[0] - error_array[4]));

        if (controller == 1)
        {
            // P controller
            control = fix2int15(proportional);
        }
        else if (controller == 2)
        {
            // PI controller
            control = fix2int15(proportional + integral_part);
        }
        else if (controller == 3)
        {
            // PD controller
            control = fix2int15(proportional + derivative);
        }
        else if (controller == 4)
        {
            // PID controller
            control = fix2int15(proportional + derivative + integral_part);
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
    sprintf(screentext, "0");
    setCursor(50, 150);
    writeString(screentext);
    sprintf(screentext, "5000");
    setCursor(45, 75);
    writeString(screentext);
    sprintf(screentext, "-5000");
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

            // Plot Angle
            drawPixel(xcoord, 430 - (int)((float)(fix2int15(accel_angle)) * 150 / 180), WHITE);
            drawPixel(xcoord, 430 - (int)((float)(fix2int15(gyro_angle_delta)) * 150 / 180), RED);
            drawPixel(xcoord, 430 - (int)((float)(fix2int15(complementary_angle)) * 150 / 180), GREEN);

            // Plot Duty Cycle
            drawPixel(xcoord, 155 - (int)((float)(fix2int15(proportional))*75 / 5000), WHITE);
            drawPixel(xcoord, 155 - (int)((float)(fix2int15(derivative))*75 / 5000), RED);
            drawPixel(xcoord, 155 - (int)((float)(fix2int15(integral_part))*75 / 5000), CYAN);
            drawPixel(xcoord, 155 - (int)((float)(control)*75 / 5000), GREEN);

            if (counter_0 > 30)
            {
                fillRect(500, 10, 600, 80, BLACK);
                sprintf(screentext, "Duty Cycle=%d", control);
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 10);
                writeString(screentext);
                sprintf(screentext, "Desired Angle = %d", fix2int15(angle_reference));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 20);
                writeString(screentext);
                sprintf(screentext, "Current angle = %d", fix2int15(complementary_angle));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 30);
                writeString(screentext);
                sprintf(screentext, "Error = %d", fix2int15(error));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 40);
                writeString(screentext);
                sprintf(screentext, "kp =%d", fix2int15(kp));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 50);
                writeString(screentext);
                sprintf(screentext, "ki=%d", fix2int15(ki));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 60);
                writeString(screentext);
                sprintf(screentext, "kd=%d", fix2int15(kd));
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(500, 70);
                writeString(screentext);

                // fillRect(70, 150, 50, 50, BLACK);
                sprintf(screentext, "Proportional = White");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 50);
                writeString(screentext);
                sprintf(screentext, "Integral = Cyan");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 60);
                writeString(screentext);
                sprintf(screentext, "Derivative = Red");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 70);
                writeString(screentext);
                sprintf(screentext, "Duty Cycle = Green");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 80);
                writeString(screentext);

                // fillRect(450, 250, 500, 280, BLACK);
                sprintf(screentext, "Accel angle = White");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 260);
                writeString(screentext);
                sprintf(screentext, "Gyro Angle = Red");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 270);
                writeString(screentext);
                sprintf(screentext, "Comp Angle = Green");
                setTextColor(WHITE);
                setTextSize(1);
                setCursor(100, 280);
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

        // bool button_read = gpio_get(BUTTON);
        // printf("BUTTON: %d\n", button_read);
        if (!gpio_get(BUTTON))
        {
            angle_reference = 0;
            pressed = true;
            t1 = time_us_32() + 5000000;
            t2 = t1 + 5000000;
            t3 = t2 + 5000000;
        }

        if (gpio_get(BUTTON) && pressed)
        {
            uint32_t time = time_us_32();
            if (time < t1)
            {
                angle_reference = int2fix15(90);
            } else if (time < t2) {
                angle_reference = int2fix15(120);
            } else if (time < t3) {
                angle_reference = int2fix15(60);
            } else {
                angle_reference = int2fix15(90);
                pressed = false;
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

    // wait for 1 sec
    PT_YIELD_usec(1000000);

    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write;

    // My own
    static char cmd[20], arg1[10];
    static char *token;
    while (1)
    {
        // print prompt
        sprintf(pt_serial_out_buffer, "Enter Command> ");
        // spawn a thread to do the non-blocking write
        serial_write;

        // spawn a thread to do the non-blocking serial read
        serial_read;

        // tokenize
        token = strtok(pt_serial_in_buffer, " ");
        strcpy(cmd, token);
        token = strtok(NULL, " ");
        strcpy(arg1, token);

        // parse by command
        if (strcmp(cmd, "help") == 0)
        {
            // List commands
            printf("timestep\n\r");
            printf("dutycycle -- (0-5000 only)\n\r");
            printf("desiredangle\n\r");
            printf("setcontrol none\n\r");
            printf("setcontrol P\n\r");
            printf("setcontrol PI\n\r");
            printf("setcontrol PD\n\r");
            printf("setcontrol PID\n\r");
            printf("kp\n\r");
            printf("ki\n\r");
            printf("kd\n\r");
            printf("integralwindup\n\r");
            printf("stop\n\r");
        }
        else if (strcmp(cmd, "timestep") == 0)
        {
            if (arg1 != NULL)
            {
                threshold = atoi(arg1);
            }
        }
        else if (strcmp(cmd, "dutycycle") == 0)
        {
            // printf("HELLOOOOO???");
            if (arg1 != NULL)
            {
                // printf("?????????");
                control = atoi(arg1);
                // printf("control = %d\n", control);
            }
        }
        else if (strcmp(cmd, "desiredangle") == 0)
        {
            if (arg1 != NULL)
            {
                angle_reference = float2fix15(atof(arg1));
                integral = 0;
            }
        }
        else if (strcmp(cmd, "setcontrol") == 0)
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
                // printf("controller =%d", controller);
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
                // if (ki > 1) {
                //     integral_wind_up = divfix(float2fix15(5000),ki);
                // } else {
                //     integral_wind_up = int2fix15(5000);
                // }
            }
        }
        else if (strcmp(cmd, "kd") == 0)
        {
            if (arg1 != NULL)
            {
                kd = float2fix15(atof(arg1));
            }
        } 
        else if (strcmp(cmd, "integralwindup") == 0)
        {
            if (arg1 != NULL)
            {
                integral_wind_up = float2fix15(atof(arg1));
            }
        }
        else if (strcmp(cmd, "stop") == 0)
        {
            controller = 0;
            control = 0;
        }
        else {
            printf("Huh?\n\r");
        }
    }
    PT_END(pt);
}

// This thread runs on core 0
// static PT_THREAD (protothread_button(struct pt *pt))
// {
//     // Indicate thread beginning
//     PT_BEGIN(pt) ;

//     while(1) {

//         if (gpio_get(BUTTON))
//         {
//             pressed = true;
//             t1 = time_us_32() + 5000000;
//             t2 = t1 + 5000000;
//             t3 = t2 + 5000000;
//         }

//         if (pressed)
//         {
//             uint32_t time = time_us_32();
//             if (time < t1)
//             {
//                 angle_reference = int2fix15(90);
//             } else if (time < t2) {
//                 angle_reference = int2fix15(120);
//             } else if (time < t3) {
//                 angle_reference = int2fix15(60);
//             } else {
//                 angle_reference = 0;
//                 pressed = false;
//             }
//         }

//         PT_YIELD_usec(30000) ;
//     }
//     // Indicate thread end
//     PT_END(pt) ;
// }

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

    // Button
    gpio_init(BUTTON);
    gpio_set_dir(BUTTON, GPIO_IN);
    gpio_pull_up(BUTTON);

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
    // pt_add_thread(protothread_button);
    pt_schedule_start;
}

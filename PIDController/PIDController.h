#pragma once
#include <Arduino.h>
#include <Common/Common.h>

/* ===============================================================
 *
 *  PID_VALIDATE_INPUT 
 *  - true/false
 *  - To validate input values.
 *
 *  PID_REGULATE_OUTPUT  
 *  - true/false
 *  - Make sure output is in the given range.
 *
 *  PID_DEAD_LOCK :
 *  - number of millis 
 *  - If output reaches limits continuously for more 
 *    than X ms, then set PID output zero.
 * 
 * ============================================================== */
#define PID_VALIDATE_INPUT    true
#define PID_REGULATE_OUTPUT   true
#define PID_DEAD_LOCK         5000000


struct pid_param_t {
  double kp;                    // Kp value
  double ki;                    // Ki value
  double kd;                    // Kd value
  double min_input;             // Min Input - cannot handle outside the range, set PID output zero.
  double max_input;             // Max Input - cannot handle outside the range, set PID output zero.
  double min_output;            // Min Output value, default 0
  double max_output;            // Max Output value, default 1
};


class PIDController {
  public:
    PIDController(pid_param_t param);
    void init();
    double calculate(double set_point, double input);
    double calculate(double error);
    void set_input_limits(double min_input, double max_input);
    void set_output_limits(double min_output, double max_output);
    void set_tunings(double kp, double ki, double kd);
    void set_direction(int direction);
    void reset();
    void get_output(double *p_output, double *i_output, double *d_output );
    pid_param_t get_param(void);

  private:
    pid_param_t param;

    // Input / Output
    double set_point;              // set point of PID
    double input;                  // Sensor input
    double error;                  // the current error
    double output;                 // PID output
    double output_center_point;    // PID output center point

    // Part of each PID error
    double last_error;             // capture last errors 
    double p_error;                // current error       
    double d_error;                // derivertive of errors
    double i_error;                // integration of errors
    
    // Part of each PID outputs
    double p_output;               // P output 
    double i_output;               // I output 
    double d_output;               // D output 

    unsigned long sample_time;
    boolean output_at_limit;
    unsigned long output_at_limit_time;
    unsigned long elapsed_time;

    boolean validate_input(void);
    void zero_output(void); 
    void regulate_output(void);


};
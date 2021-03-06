#include <PIDController/PIDController.h>

/*
 ***************************************************************
 * 
 *  Contructor
 * 
 **************************************************************
 */

PIDController::PIDController(pid_param_t param) {
  this->param.min_input              = param.min_input;
  this->param.max_input              = param.max_input;
  this->param.min_output             = param.min_output;
  this->param.max_output             = param.max_output; 
  this->output_center_point          = (this->param.min_output +  this->param.max_output)/2.0;
}


/***************************************************************
 * 
 *  All init methods
 * 
 **************************************************************/

void PIDController::init() {
  this->reset();
}; 



/***************************************************************
 * 
 *  Set parameters
 * 
 **************************************************************/

void PIDController::set_input_limits(double min_input, double max_input) {
  this->param.min_input = min_input;
  this->param.max_input = max_input;
};

void PIDController::set_output_limits(double min_output, double max_output) {
  this->param.min_output = min_output;
  this->param.max_output = max_output;
  this->output_center_point = (this->param.min_output +  this->param.max_output)/2.0;
};

/***************************************************************
 * 
 *  Set tunings at run-time
 * 
 **************************************************************/
void PIDController::set_tunings(double kp, double ki, double kd) {
  this->param.kp  = kp;
  this->param.ki  = ki;
  this->param.kd  = kd;
  this->reset();
};




pid_param_t PIDController::get_param() {
  return this->param;
};


boolean PIDController::validate_input() {
  // By pass input validation
  if (! PID_VALIDATE_INPUT) {
    return true;
  };

  // By pass input validation if setting invalid
  if (this->param.min_input == 0.0 && this->param.max_input == 0.0) {
    return true;
  };
  
  // By pass input validation if range invalid
  if (this->param.min_input >= this->param.max_input ) {
    return true;
  };

  // Input out of range
  if ( ((this->input < this->param.min_input) )  || (this->input > this->param.max_input) ) {
    return false;
  };
  
  // default 
  return true;
}


void PIDController::regulate_output() {
  // By pass output regulation
  if (!PID_REGULATE_OUTPUT) {
    this->output_at_limit = false;
    return;
  };

  // By pass output regulation if invalid setting
  if (this->param.min_output == 0.0 && this->param.max_output == 0.0) {
    this->output_at_limit = false;
    return;
  };
  
  // By pass output regulation if invalid range
  if (this->param.min_output >= this->param.max_output ) {
    this->output_at_limit = false;
    return;
  };

  // Output regulation
  if (this->output  < this->param.min_output) {  
    this->output_at_limit = true;
    this->output  =  this->param.min_output;
  } else if (this->output  > this->param.max_output) {  
    this->output_at_limit = true;
    this->output  =  this->param.max_output;
  } else {
    this->output_at_limit_time = millis();
    this->output_at_limit = false;
  };

  // When reached limit for more than x ms, lock the PID
  if ( (PID_DEAD_LOCK >0) && (this->output_at_limit) && (millis() - this->output_at_limit_time>PID_DEAD_LOCK) ) {
    this->zero_output();
  };
};

void PIDController::zero_output() {
  this->output = this->output_center_point;  
};

/***************************************************************
 * 
 *  Reset PID, cleanup all errors and start over.
 * 
 **************************************************************/
void PIDController::reset() {
  this->p_error  = 0.0;
  this->i_error  = 0.0;
  this->d_error  = 0.0;
  this->p_output = 0.0;
  this->i_output = 0.0;
  this->d_output = 0.0;
  this->output_center_point = (this->param.min_output +  this->param.max_output)/2.0;
  this->output = this->output_center_point;
  this->sample_time = millis();
  this->output_at_limit = false;
  this->output_at_limit_time = millis();
};


/***************************************************************
 * 
 *  Calculate PID output
 * 
 **************************************************************/
double PIDController::calculate(double error) {
  /*
    this is to leave program outside to calculate error,
    then assume the set_point = 0
  */
     
  this->error = error;
  return this->calculate(0, this->error);
};

double PIDController::calculate(double set_point, double input) {
  this->set_point = set_point;
  this->input = input;

  long now = millis();
  this->elapsed_time = now -  this->sample_time;
  this->p_error = this->set_point - this->input;
  
  if ( this->elapsed_time>0 ) {
    // validate input
    if (this->validate_input()) {
      
      this->i_error     += this->p_error * this->elapsed_time;
      this->d_error     =  (this->p_error - this->last_error) / this->elapsed_time;
      this->last_error  =  this->p_error;
      this->p_output    =  this->param.kp * this->p_error; 
      this->i_output    =  this->param.ki * this->i_error;
      this->d_output    =  this->param.kd * this->d_error;

      // Overall output, maps to output range linearly.
      this->output = this->p_output  + this->i_output  +  this->d_output + this->output_center_point;

      // Regulate output
      this->regulate_output();
    } else {
      this->zero_output();
    };
    this->sample_time =  now;
  };
  return this->output;
};

void PIDController::get_output(double *p_output, double *i_output, double *d_output ) {
  *p_output = this->p_output;
  *i_output = this->i_output;
  *d_output = this->d_output;
};

/*
  report.c - reporting and messaging methods
  Part of Grbl

  The MIT License (MIT)

  GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2012 Sungeun K. Jeon

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/
/* 
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such 
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a 
  different style feedback is desired (i.e. JSON), then a user can change these following 
  methods to accomodate their needs.
*/

//#include <avr/pgmspace.h>
//#include "report.h"
//#include "print.h"
//#include "settings.h"
//#include "nuts_bolts.h"
//#include "gcode.h"
//#include "coolant_control.h"

#include "include.h"

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an 
// 'error:'  to indicate some error event with the line or some critical system error during 
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
// NOTE: In silent mode, all error codes are greater than zero.
// TODO: Install silent mode to return only numeric values, primarily for GUIs.
void report_status_message(uint8_t status_code) 
{
  if (status_code == 0) { // STATUS_OK
    printPgmString((const char *)("ok\r\n"));
  } else {
    printPgmString((const char *)("error: "));
    switch(status_code) {          
      case STATUS_BAD_NUMBER_FORMAT:
      printPgmString((const char *)("Bad number format")); break;
      case STATUS_EXPECTED_COMMAND_LETTER:
      printPgmString((const char *)("Expected command letter")); break;
      case STATUS_UNSUPPORTED_STATEMENT:
      printPgmString((const char *)("Unsupported statement")); break;
      case STATUS_ARC_RADIUS_ERROR:
      printPgmString((const char *)("Invalid radius")); break;
      case STATUS_MODAL_GROUP_VIOLATION:
      printPgmString((const char *)("Modal group violation")); break;
      case STATUS_INVALID_STATEMENT:
      printPgmString((const char *)("Invalid statement")); break;
      case STATUS_SETTING_DISABLED:
      printPgmString((const char *)("Setting disabled")); break;
      case STATUS_SETTING_VALUE_NEG:
      printPgmString((const char *)("Value < 0.0")); break;
      case STATUS_SETTING_STEP_PULSE_MIN:
      printPgmString((const char *)("Value < 3 usec")); break;
      case STATUS_SETTING_READ_FAIL:
      printPgmString((const char *)("EEPROM read fail. Using defaults")); break;
      case STATUS_IDLE_ERROR:
      printPgmString((const char *)("Busy or queued")); break;
      case STATUS_ALARM_LOCK:
      printPgmString((const char *)("Alarm lock")); break;
      case STATUS_OVERFLOW:
      printPgmString((const char *)("Line overflow")); break;
    }
    printPgmString((const char *)("\r\n"));
  }
}

// Prints alarm messages.
void report_alarm_message(int8_t alarm_code)
{
  printPgmString((const char *)("ALARM: "));
  switch (alarm_code) {
    case ALARM_HARD_LIMIT: 
    printPgmString((const char *)("Hard limit")); break;
    case ALARM_ABORT_CYCLE: 
    printPgmString((const char *)("Abort during cycle")); break;
  }
  printPgmString((const char *)(". MPos?\r\n"));
  delay_ms(500); // Force delay to ensure message clears serial write buffer.
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
// TODO: Install silence feedback messages option in settings
void report_feedback_message(uint8_t message_code)
{
  printPgmString((const char *)("["));
  switch(message_code) {
    case MESSAGE_CRITICAL_EVENT:
    printPgmString((const char *)("Reset to continue")); break;
    case MESSAGE_ALARM_LOCK:
    printPgmString((const char *)("'$H'|'$X' to unlock")); break;
    case MESSAGE_ALARM_UNLOCK:
    printPgmString((const char *)("Caution: Unlocked")); break;
    case MESSAGE_ENABLED:
    printPgmString((const char *)("Enabled")); break;
    case MESSAGE_DISABLED:
    printPgmString((const char *)("Disabled")); break;    
  }
  printPgmString((const char *)("]\r\n"));
}


// Welcome message
void report_init_message()
{
  printPgmString((const char *)("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n"));
}

// Grbl help message
void report_grbl_help() {
  printPgmString((const char *)("$$ (view Grbl settings)\r\n"
                      "$# (view # parameters)\r\n"
                      "$G (view parser state)\r\n"
                      "$N (view startup blocks)\r\n"
                      "$x=value (save Grbl setting)\r\n"
                      "$Nx=line (save startup block)\r\n"
                      "$C (check gcode mode)\r\n"
                      "$X (kill alarm lock)\r\n"
                      "$H (run homing cycle)\r\n"
                      "~ (cycle start)\r\n"
                      "! (feed hold)\r\n"
                      "? (current status)\r\n"
                      "ctrl-x (reset Grbl)\r\n"));
}

// Grbl global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void report_grbl_settings() {
  printPgmString((const char *)("$0=")); printFloat(settings.steps_per_mm[X_AXIS]);
  printPgmString((const char *)(" (x, step/mm)\r\n$1=")); printFloat(settings.steps_per_mm[Y_AXIS]);
  printPgmString((const char *)(" (y, step/mm)\r\n$2=")); printFloat(settings.steps_per_mm[Z_AXIS]);
  printPgmString((const char *)(" (z, step/mm)\r\n$3=")); printInteger(settings.pulse_microseconds);
  printPgmString((const char *)(" (step pulse, usec)\r\n$4=")); printFloat(settings.default_feed_rate);
  printPgmString((const char *)(" (default feed, mm/min)\r\n$5=")); printFloat(settings.default_seek_rate);
  printPgmString((const char *)(" (default seek, mm/min)\r\n$6=")); printInteger(settings.invert_mask); 
  printPgmString((const char *)(" (step port invert mask, int:")); print_uint8_base2(settings.invert_mask);  
  printPgmString((const char *)(")\r\n$7=")); printInteger(settings.stepper_idle_lock_time);
  printPgmString((const char *)(" (step idle delay, msec)\r\n$8=")); printFloat(settings.acceleration/(60*60)); // Convert from mm/min^2 for human readability
  printPgmString((const char *)(" (acceleration, mm/sec^2)\r\n$9=")); printFloat(settings.junction_deviation);
  printPgmString((const char *)(" (junction deviation, mm)\r\n$10=")); printFloat(settings.mm_per_arc_segment);
  printPgmString((const char *)(" (arc, mm/segment)\r\n$11=")); printInteger(settings.n_arc_correction);
  printPgmString((const char *)(" (n-arc correction, int)\r\n$12=")); printInteger(settings.decimal_places);
  printPgmString((const char *)(" (n-decimals, int)\r\n$13=")); printInteger(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
  printPgmString((const char *)(" (report inches, bool)\r\n$14=")); printInteger(bit_istrue(settings.flags,BITFLAG_AUTO_START));
  printPgmString((const char *)(" (auto start, bool)\r\n$15=")); printInteger(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
  printPgmString((const char *)(" (invert step enable, bool)\r\n$16=")); printInteger(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
  printPgmString((const char *)(" (hard limits, bool)\r\n$17=")); printInteger(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
  printPgmString((const char *)(" (homing cycle, bool)\r\n$18=")); printInteger(settings.homing_dir_mask);
  printPgmString((const char *)(" (homing dir invert mask, int:")); print_uint8_base2(settings.homing_dir_mask);  
  printPgmString((const char *)(")\r\n$19=")); printFloat(settings.homing_feed_rate);
  printPgmString((const char *)(" (homing feed, mm/min)\r\n$20=")); printFloat(settings.homing_seek_rate);
  printPgmString((const char *)(" (homing seek, mm/min)\r\n$21=")); printInteger(settings.homing_debounce_delay);
  printPgmString((const char *)(" (homing debounce, msec)\r\n$22=")); printFloat(settings.homing_pulloff);
  printPgmString((const char *)(" (homing pull-off, mm)\r\n")); 
}


// Prints gcode coordinate offset parameters
void report_gcode_parameters()
{
  float coord_data[N_AXIS];
  uint8_t coord_select, i;
  for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++) { 
    if (!(settings_read_coord_data(coord_select,coord_data))) { 
      report_status_message(STATUS_SETTING_READ_FAIL); 
      return;
    } 
    printPgmString((const char *)("[G"));
    switch (coord_select) {
      case 0: printPgmString((const char *)("54:")); break;
      case 1: printPgmString((const char *)("55:")); break;
      case 2: printPgmString((const char *)("56:")); break;
      case 3: printPgmString((const char *)("57:")); break;
      case 4: printPgmString((const char *)("58:")); break;
      case 5: printPgmString((const char *)("59:")); break;
      case 6: printPgmString((const char *)("28:")); break;
      case 7: printPgmString((const char *)("30:")); break;
      // case 8: printPgmString((const char *)("92:")); break; // G92.2, G92.3 not supported. Hence not stored.  
    }           
    for (i=0; i<N_AXIS; i++) {
      if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) { printFloat(coord_data[i]*INCH_PER_MM); }
      else { printFloat(coord_data[i]); }
      if (i < (N_AXIS-1)) { printPgmString((const char *)(",")); }
      else { printPgmString((const char *)("]\r\n")); }
    } 
  }
  printPgmString((const char *)("[G92:")); // Print G92,G92.1 which are not persistent in memory
  for (i=0; i<N_AXIS; i++) {
    if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) { printFloat(gc.coord_offset[i]*INCH_PER_MM); }
    else { printFloat(gc.coord_offset[i]); }
    if (i < (N_AXIS-1)) { printPgmString((const char *)(",")); }
    else { printPgmString((const char *)("]\r\n")); }
  } 
}


// Print current gcode parser mode state
void report_gcode_modes()
{
  switch (gc.motion_mode) {
    case MOTION_MODE_SEEK : printPgmString((const char *)("[G0")); break;
    case MOTION_MODE_LINEAR : printPgmString((const char *)("[G1")); break;
    case MOTION_MODE_CW_ARC : printPgmString((const char *)("[G2")); break;
    case MOTION_MODE_CCW_ARC : printPgmString((const char *)("[G3")); break;
    case MOTION_MODE_CANCEL : printPgmString((const char *)("[G80")); break;
  }

  printPgmString((const char *)(" G"));
  printInteger(gc.coord_select+54);
  
  if (gc.plane_axis_0 == X_AXIS) {
    if (gc.plane_axis_1 == Y_AXIS) { printPgmString((const char *)(" G17")); }
    else { printPgmString((const char *)(" G18")); }
  } else { printPgmString((const char *)(" G19")); }
  
  if (gc.inches_mode) { printPgmString((const char *)(" G20")); }
  else { printPgmString((const char *)(" G21")); }
  
  if (gc.absolute_mode) { printPgmString((const char *)(" G90")); }
  else { printPgmString((const char *)(" G91")); }
  
  if (gc.inverse_feed_rate_mode) { printPgmString((const char *)(" G93")); }
  else { printPgmString((const char *)(" G94")); }
    
  switch (gc.program_flow) {
    case PROGRAM_FLOW_RUNNING : printPgmString((const char *)(" M0")); break;
    case PROGRAM_FLOW_PAUSED : printPgmString((const char *)(" M1")); break;
    case PROGRAM_FLOW_COMPLETED : printPgmString((const char *)(" M2")); break;
  }

  switch (gc.spindle_direction) {
    case 1 : printPgmString((const char *)(" M3")); break;
    case -1 : printPgmString((const char *)(" M4")); break;
    case 0 : printPgmString((const char *)(" M5")); break;
  }
  
  switch (gc.coolant_mode) {
    case COOLANT_DISABLE : printPgmString((const char *)(" M9")); break;
    case COOLANT_FLOOD_ENABLE : printPgmString((const char *)(" M8")); break;
    #ifdef ENABLE_M7
      case COOLANT_MIST_ENABLE : printPgmString((const char *)(" M7")); break;
    #endif
  }
  
  printPgmString((const char *)(" T"));
  printInteger(gc.tool);
  
  printPgmString((const char *)(" F"));
  if (gc.inches_mode) { printFloat(gc.feed_rate*INCH_PER_MM); }
  else { printFloat(gc.feed_rate); }

  printPgmString((const char *)("]\r\n"));
}

// Prints specified startup line
void report_startup_line(uint8_t n, char *line)
{
  printPgmString((const char *)("$N")); printInteger(n);
  printPgmString((const char *)("=")); printString(line);
  printPgmString((const char *)("\r\n"));
}

 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram 
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly, 
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void report_realtime_status()
{
  // **Under construction** Bare-bones status report. Provides real-time machine position relative to 
  // the system power on location (0,0,0) and work coordinate position (G54 and G92 applied). Eventually
  // to be added are distance to go on block, processed block id, and feed rate. Also a settings bitmask
  // for a user to select the desired real-time data.
  uint8_t i;
  int32_t current_position[3]; // Copy current state of the system position variable
  float print_position[3];

   memcpy(current_position,sys.position,sizeof(sys.position));
  // Report current machine state
  switch (sys.state) {
    case STATE_IDLE: printPgmString((const char *)("<Idle")); break;
//    case STATE_INIT: printPgmString((const char *)("[Init")); break; // Never observed
    case STATE_QUEUED: printPgmString((const char *)("<Queue")); break;
    case STATE_CYCLE: printPgmString((const char *)("<Run")); break;
    case STATE_HOLD: printPgmString((const char *)("<Hold")); break;
    case STATE_HOMING: printPgmString((const char *)("<Home")); break;
    case STATE_ALARM: printPgmString((const char *)("<Alarm")); break;
    case STATE_CHECK_MODE: printPgmString((const char *)("<Check")); break;
  }
 
  // Report machine position
  printPgmString((const char *)(",MPos:")); 
  for (i=0; i<= 2; i++) {
    print_position[i] = current_position[i]/settings.steps_per_mm[i];
    if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) { print_position[i] *= INCH_PER_MM; }
    printFloat(print_position[i]);
    printPgmString((const char *)(","));
  }
  
  // Report work position
  printPgmString((const char *)("WPos:")); 
  for (i=0; i<= 2; i++) {
    if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
      print_position[i] -= (gc.coord_system[i]+gc.coord_offset[i])*INCH_PER_MM;
    } else {
      print_position[i] -= gc.coord_system[i]+gc.coord_offset[i];
    }
    printFloat(print_position[i]);
    if (i < 2) { printPgmString((const char *)(",")); }
  }
    
  printPgmString((const char *)(">\r\n"));
}

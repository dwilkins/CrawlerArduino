#include <Arduino.h>
#include "CrawlerCommand.hpp"
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include "main.h"

CrawlerCommand::CrawlerCommand(const char *command,
                               int num_parameters,
                               int parameter_types[]) {
  this->command = command;
  this->num_parameters = num_parameters;
  for(int i = 0;i < this->num_parameters && i < CC_MAX_PARAMETERS;i++) {
    this->parameters[i].param_type = parameter_types[i];
  }
}

bool CrawlerCommand::is_match(char* buffer) {
  static char our_buffer[CC_SERIAL_BUFFER_SIZE];
  char *buf_ptr = our_buffer;
  int token_num = 0;
  char *tokens[10];
  bool retval = false;
  strncpy(our_buffer,buffer,CC_SERIAL_BUFFER_SIZE);
  while(isspace(*buf_ptr)) {buf_ptr++;}
  while(buf_ptr && *buf_ptr && (tokens[token_num] = strsep(&buf_ptr," ")) && (token_num < 10)) {
    token_num++;
  }
  if(strcasecmp(tokens[0],this->command)) {
    return false;
  } else {
    myPrint("Valid Command: ");
    myPrint(tokens[0]);
    myPrint(" :for: ");
    myPrintln(this->command);
  }
  for(int t = 0 ; t < token_num; t++) {
    myPrint("Token: ");
    myPrintln(tokens[t]);
  }


  if(!strcasecmp(tokens[0],command) && token_num > num_parameters) {
    for(int p = 0; p < num_parameters; p++) {
      switch (parameters[p].param_type) {
      case PT_INTEGER:
        parameters[p].val.i = atoi(tokens[p + 1]);
        break;
      case PT_FLOAT:
        parameters[p].val.f = atof(tokens[p + 1]);
        break;
      case PT_STRING:
        strncpy(parameters[p].val.s,tokens[p + 1],CC_MAX_PARAMETER_SIZE);
        break;
      }
    }
    retval = true;
  } else {
    myPrint("Sadly, parameter count mismatch - Got ");
    myPrint(token_num, DEC);
    myPrint(" tokens instead of ");
    myPrintln(this->num_parameters + 1,DEC);
  }
  return retval;
}

bool CrawlerCommand::execute() {
  if(command_function) {
    return command_function(num_parameters,parameters);
  }
  return false;
}

const char * CrawlerCommand::get_command() {
  return command;
}

int CrawlerCommand::get_num_parameters() {
  return num_parameters;
}

const char ** CrawlerCommand::get_parameter_type_names() {
  static const char *types[CC_MAX_PARAMETERS];
  for(int i = 0;i < CC_MAX_PARAMETERS ; i++) {
    switch (parameters[i].param_type) {
    case PT_INTEGER:
      types[i] = "Integer";
      break;
    case PT_FLOAT:
      types[i] = "Float";
      break;
    case PT_STRING:
      types[i] = "String";
      break;
    }
  }
  return types;
}


void CrawlerCommand::set_command_function(int (*fn)(int,parameter_t *)) {
  command_function = fn;
}


const char valid_characters[UCHAR_MAX] = {
// NUL,SOH,STX,ETX,EOT,ENQ,ACK,BEL, BS, HT, LF, VT, FF, CR, SO, SI,DLE,DC1,DC2,DC3,
     0,  0,  0,  0,  1,  0,  0,  0,  1,  0,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,
// DC4,NAK,SYN,ETB,CAN, EM,SUB,ESC, FS, GS, RS, US,   ,  !,  ",  #,  $,  %,  &,  ',
     0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
//   (,  ),  *,  +,',',  -,  .,  /,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  :,  ;,
     0,  0,  0,  0,  0,  0,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,
//   <,  =,  >,  ?,  @,  A,  B,  C,  D,  E,  F,  G,  H,  I,  J,  K,  L,  M,  N,  O,
     0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
//   P,  Q,  R,  S,  T,  U,  V,  W,  X,  Y,  Z,  [,  \,  ],  ^,  _,  `,  a,  b,  c,
     1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  1,  1,  1,
//   d,  e,  f,  g,  h,  i,  j,  k,  l,  m,  n,  o,  p,  q,  r,  s,  t,  u,  v,  w,
     1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
//   x,  y,  z,  {,  |,  },  ~
     1,  1,  1,  0,  0,  0,  0
// Everything else should be initialized to 0
};


CrawlerCommand::command_buffer_t * CrawlerCommand::read_command() {
  static char serial_buffer[CC_SERIAL_BUFFER_SIZE] = {};
  static int serial_buffer_next_char = 0;
  static bool end_of_command = false;
  static command_buffer_t cb;
  command_buffer_t *ret_val = NULL;
  if(end_of_command) {
    memset(serial_buffer,0,sizeof(serial_buffer));
    serial_buffer_next_char = 0;
    end_of_command = false;
  }
  if(Serial.available() == 0) {
    return NULL;
  }
 while (Serial.available() > 0) {
    int inByte = Serial.read();
    if(inByte > UCHAR_MAX || !valid_characters[inByte]) {
      continue;
    }
    myPrint((char)inByte);
    switch(inByte) {
      case '\n':
      case '\r':
        end_of_command = true;
        myPrintln("");
      break;
      case '\b':
        if(serial_buffer_next_char > 0) {
          serial_buffer[serial_buffer_next_char--] = 0;
        }
      break;
      default:
      serial_buffer[serial_buffer_next_char++] = inByte;
      serial_buffer[serial_buffer_next_char] = 0;
      break;
    }
  }
  // If end of command, rip through the command buffer and send back pointers to
  // each of the commands.   Commands may be separated by semi-colon
  if(end_of_command && serial_buffer_next_char > 0) {
    char *next_command = &serial_buffer[0];
    char *buffer_end = &serial_buffer[sizeof(serial_buffer)-1];
    cb.num_commands = 0;
    while(next_command && next_command < buffer_end && *next_command) {
      for(;next_command < buffer_end && (*next_command == ';' || isspace(*next_command));next_command++) {
        *next_command = 0; // NULL terminate previous command by replacing ; and space with null
      }
      cb.command_strings[cb.num_commands++] = next_command;
      next_command = strchr(next_command,';');
    }
    ret_val = &cb;
  }
  return ret_val;
}

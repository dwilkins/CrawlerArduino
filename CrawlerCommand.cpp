#include "CrawlerCommand.hpp"
#include <string.h>


CrawlerCommand::CrawlerCommand(const char *command,
                               int num_parameters,
                               int parameter_types[]) {
  strncpy(this->command,command,CC_STATUS_BUFFER_SIZE);
  this->num_parameters = num_parameters;
  for(int i = 0;i < this->num_parameters && i < CC_MAX_PARAMETERS;i++) {
    this->parameter_types[i] = parameter_types[i];
  }
}

void CrawlerCommand::set_command_function(int (*fn)(int,void **)) {
  command_function = fn;
}


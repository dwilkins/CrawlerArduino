
#define CC_MAX_PARAMETERS 3
#define CC_STATUS_BUFFER_SIZE 50
#define CC_MAX_PARAMETER_SIZE 10
#define CC_SERIAL_BUFFER_SIZE 255

class CrawlerCommand {
public:
  // constructors:
  CrawlerCommand(const char *command, int parameters,int parameter_types[]);
  bool is_match(char *buffer);
  bool execute();


  const char* get_last_status();

  int get_num_matches();
  const char * get_command();
  int get_num_parameters();
  int *get_parameter_types();
  const char **get_parameter_type_names();


  enum {PT_INTEGER, PT_FLOAT, PT_STRING};
  struct parameter_t {
    int param_type;
    union val_t {
      int i;
      float f;
      char s[CC_MAX_PARAMETER_SIZE];
    } val;
  } parameter;

  void set_command_function(int (*fn)(int,parameter_t *));

  struct command_buffer_t {
    int num_commands;
    char *command_strings[20];
  } command_buffer;


  static command_buffer_t * read_command();


protected:
  int num_matches;
  int (*command_function)(int,parameter_t *);
  const char *command;
  int num_parameters;
  parameter_t parameters[CC_MAX_PARAMETERS];
private:
  char status_buffer[CC_STATUS_BUFFER_SIZE];
};


#define CC_MAX_PARAMETERS 10
#define CC_STATUS_BUFFER_SIZE 128

class CrawlerCommand {
public:
  // constructors:
  CrawlerCommand(const char *command, int parameters,int parameter_types[]);
  bool is_match(const char *buffer);
  bool execute(const char *buffer);

  void set_command_function(int (*fn)(int,void **));

  const char* get_last_status();

  int get_num_matches();
  const char * get_command();
  int get_num_parameters();
  int *get_parameter_types();


  enum {PT_INTEGER, PT_FLOAT, PT_STRING};

protected:
  int num_matches;
  int (*command_function)(int,void **);
  char *command;
  int num_parameters;
  int parameter_types[CC_MAX_PARAMETERS];
private:
  char status_buffer[CC_STATUS_BUFFER_SIZE];
};

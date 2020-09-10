#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <ros/ros.h>

class exec_plugin
{
  pid_t pid;

public:
  exec_plugin()
  {
    pid_t pid = fork();
    if(pid == 0)
    {
      execlp("rqt", "rqt", "--standalone", "surveillance_system", NULL);
    }
  }
  ~exec_plugin() {}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plugin");
  exec_plugin plugin;
  ros::spin();
  return 0;
}

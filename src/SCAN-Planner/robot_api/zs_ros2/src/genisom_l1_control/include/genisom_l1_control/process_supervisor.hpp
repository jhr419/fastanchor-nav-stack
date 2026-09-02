#ifndef GENISOM_L1_CONTROL__PROCESS_SUPERVISOR_HPP_
#define GENISOM_L1_CONTROL__PROCESS_SUPERVISOR_HPP_

namespace genisom_l1_control
{

using WorkerMain = int (*)(int argc, char * argv[]);

// 在独立子进程中运行官方 SDK，确保构造期卡死仍可被外层进程回收。
int run_with_shutdown_supervisor(int argc, char * argv[], WorkerMain worker_main);

// 工作进程内部请求退出时通知监督进程，确保 SDK 析构卡死仍会被限时回收。
void request_supervised_shutdown();

}  // 命名空间 genisom_l1_control

#endif

# 工程规范化设计

## 目标

工程以 `user/README.md` 作为唯一用户操作入口。用户不需要了解源码目录即可完成环境加载、构建、启动、停止、测试和数据回放。

## 目录职责

- `src/`：ROS 2 包和业务实现。
- `user/`：环境、构建、启动、停止、测试和数据操作。
- `maps/`：运行地图。
- `docs/`：架构、问题分析和开发设计记录。
- `build/`、`install/`、`log/`：colcon 生成目录。

ROS 包的 `launch/` 与 `config/` 保留在各包内。这样 `ament` 安装后仍可通过包共享目录定位资源，避免在根目录复制配置并产生两套来源。

## 启动设计

`start_all.sh` 仅负责编排终端，不直接承载 ROS 业务命令。每个标签页调用对应的单模块脚本，因此单模块和完整系统使用同一套参数、环境和依赖检查。

```text
01 雷达
  -> 02 LIO 与 FastAnchor
  -> 03 FastPlanner、SCAN 与控制
  -> 04 底盘速度桥
  -> 05 RViz（可选）
```

后级脚本通过 ROS 图等待前置 Topic。等待逻辑集中在 `user/lib/common.sh`，超时由 `NAV_TOPIC_WAIT_TIMEOUT` 统一配置。

## 配置设计

工程级默认值集中在 `user/config.env`，命令行参数优先于默认值。路径默认使用相对工作空间的写法，运行时由公共函数解析，不依赖调用者当前目录。

底盘、规划器和定位器自身的业务参数仍由对应 ROS 包的 YAML 管理。

## 进程管理

模块启动前将 PID、Linux 进程启动时间和模块名写入 `user/.run/`。停止时同时校验 PID 与启动时间，避免 PID 被系统复用后误停无关进程。

`stop.sh` 先发送 `SIGINT` 触发 ROS 正常关闭，等待 10 秒后才对残留进程发送 `SIGTERM`。不使用按进程名匹配的宽泛停止命令。

## 兼容性

根目录 `setup.bash` 保留，但只转发到 `user/setup_env.sh`。已有使用方式继续有效，新文档统一使用 `user/` 入口。

## 验证要求

每次影响工程操作面的修改至少执行：

```bash
bash -n user/*.sh user/lib/*.sh setup.bash
./user/start_all.sh --dry-run
./user/check_env.sh
```

涉及 ROS 包或 Launch 时，还应构建受影响包、检查 Launch 参数并运行对应测试。

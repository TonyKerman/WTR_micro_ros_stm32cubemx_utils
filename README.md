![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

# micro-ROS for STM32CubeMX(vscode-eide)

在eide，stm32cubemx和freetros下使用microros

- [micro-ROS for STM32CubeMX(vscode-eide)](#micro-ros-for-stm32cubemxvscode-eide)
  - [配置stm32下位机](#配置stm32下位机)
  - [配置ros2上位机](#配置ros2上位机)
    - [前提](#前提)
    - [步骤](#步骤)
  - [可用传输方式](#可用传输方式)
    - [U(S)ART DMA](#usart-dma)
    - [U(S)ART  中断模式IT](#usart--中断模式it)
    - [USB CDC](#usb-cdc)
  - [定制 micro-ROS 库](#定制-micro-ros-库)
  - [Adding custom packages](#adding-custom-packages)
  - [参考](#参考)

## 配置stm32下位机

1. 将此存储库克隆到您的 STM32CubeMX 项目文件夹中（要求能同时看见makfile 与 WTR_micro_ros_stm32cubemx_utils）。使用cubemx生成项目,可参考`sample_project.ioc`。 
注意：
* 使用的stm32 需要支持10kb堆栈的FreRTOS Task（大概是f4吧）
* 开启UARTx-DMA，具体步骤在[可用传输方式](#可用传输方式)
* 使用FreeRTOS-v2
* `FREERTOS->Tasks and Queues`中将一个线程`Stack Size`设置为3000（3000*4bytes=12kb,要求至少10kb [Detail](.images/Set_freertos_stack.jpg)）
* 使用  Makefile 工具链

2.修改生成的 Makefile， 在 `build the application` 部分之前添加以下代码：

   <!-- # Removing heap4 manager while being polite with STM32CubeMX
   TMPVAR := $(C_SOURCES)
   C_SOURCES := $(filter-out Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c, $(TMPVAR)) -->

   ```makefile
   #######################################
   # micro-ROS addons
   #######################################
   LDFLAGS += micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
   C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

   # Add micro-ROS utils
   C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
   C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
   C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

   # Set here the custom transport implementation
   C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c

   print_cflags:
      @echo $(CFLAGS)
   ```

3. 使用Dockerfile构建并运行microros静态库生成工具：
```bash
#在micro-ros-lib-builder下
docker build -t tony/micro-ros-lib-build-humble .

```
4. 执行静态库生成工具。编译器标志将从您的 Makefile 自动检索，成功的标志是显示的项目Makefile的CFLAGS不为空
```bash
cd ... #移动到含有makefile的目录下
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=WTR_micro_ros_stm32cubemx_utils/microros_static_library tony/micro-ros-lib-build-humble 
```
5. 修改您的 `main.c` 以使用 micro-ROS。可以在 `sample_main.c` 中找到示例应用程序。
## 配置ros2上位机
### 前提
已经安装ros2 humble 环境
### 步骤
1. 安装 micro-ROS ：
```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# 创建一个新工作区
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# rosdep更新依赖（推荐使用rosdepc）
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```
2. 创建 micro-ROS agent
```bash
#创建一个微型 ROS 代理
ros2 run micro_ros_setup create_agent_ws.sh
#构建代理包
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

```
3. 运行micro-ROS 应用程序（串口）
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
# eg:ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
你可以通过`ls /dev | grep ttyUSB` 找到已连接的串口

## 可用传输方式

### U(S)ART DMA

设置步骤:
   - 在 STM32CubeMX 中启用 U(S)ART
   - 对于选定的 USART，在 `DMA Settings` 下启用 Tx 和 Rx 的 DMA
   - 将 Tx 和 Rx 的 DMA priotity设置为 `Very High``
   - 将 Rx 的 DMA 模式设置为 `Circular` ： [详细信息](.images/Set_UART_DMA1.jpg)
   - 对于选定的USART，启用 `NVIC Settings` 下的 `global interrupt` ：[详细信息](.images/Set_UART_DMA_2.jpg)

### U(S)ART  中断模式IT

设置步骤:
   - 在 STM32CubeMX 中启用 U(S)ART
   - 对于选定的 USART，启用 `NVIC Settings` 下的 `global interrupt` ： [详细信息](.images/Set_UART_IT.jpg)

### USB CDC

设置步骤:
   - 在 STM32CubeMX `Connectivity` 选项卡中启用 USB。
   - 在 `Middleware -> USB_DEVICE` 配置上选择 `Communication Device Class (Virtual Port Com)` 模式。

      **注意：micro-ROS 传输将覆盖自动生成的 `USB_DEVICE/App/usbd_cdc_if.c` 中的方法。**

## 定制 micro-ROS 库

所有 micro-ROS 配置都可以在步骤 3 之前在 `colcon.meta` 文件中完成。您可以在[中间件配置教程](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/)中找到有关如何调整库的静态内存使用情况的详细信息。

## Adding custom packages

请注意，此构建系统将考虑添加到 `microros_static_library/library_generation/extra_packages/` 的文件夹和添加到 `/microros_static_library/library_generation/extra_packages/extra_packages.repos` 的条目。

## 参考
[micro-ROS官网](https://micro.ros.org/docs/tutorials/core/overview/)
[micro-ROS/micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)



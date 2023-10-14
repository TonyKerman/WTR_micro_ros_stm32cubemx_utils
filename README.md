![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

# micro-ROS for STM32CubeMX

stm32cubemx和freetros下使用microros

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

## 配置stm32下位机(cubeMX)
1. 将此存储库克隆到您的 STM32CubeMX 项目文件夹中(主目录)
2. 配置cubeMX
* 使用的stm32 需要支持10kb堆栈的FreRTOS Task（大概是f4吧）
* 开启UARTx-DMA，具体步骤在[可用传输方式](#可用传输方式)
* 使用FreeRTOS-v2
* `FREERTOS->Tasks and Queues`中将一个线程`Stack Size`设置为3000（3000*4bytes=12kb,要求至少10kb [Detail](.images/Set_freertos_stack.jpg)）
* 使用Makefile工具链
3. 修改生成的 Makefile， 在 `build the application` 部分之前添加以下代码：

   ```makefile
   #######################################
   # micro-ROS addons
   #######################################
   LDFLAGS += WTR_micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
   C_INCLUDES += -IWTR_micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

   # Add micro-ROS utils
   C_SOURCES += WTR_micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
   C_SOURCES += WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
   C_SOURCES += WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

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
构建成功后，你可以在本地镜像列表中看到`tony/micro-ros-lib-build-humble`镜像，下次不需要再次构建
5. 执行静态库生成工具。编译器标志将从您的 Makefile 自动检索，成功的标志是显示的项目Makefile的CFLAGS不为空
```bash
cd ... #移动到含有makefile的目录下
docker trans_i2o -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=WTR_micro_ros_stm32cubemx_utils/microros_static_library tony/micro-ros-lib-build-humble 
```
生成静态库后，你可以更改工具链
### 使用Makefile工具链
直接用

#### 注意，Eide使用的不是Makefile工具链，需要额外配置
### 使用Clion(Stm32CubeIDE)工具链
1. 在Clion中新建一个cubemx工程
2. 将生成静态库后的`WTR_micro_ros_stm32cubemx_utils`文件夹和`.ioc`文件复制到新工程下
3. 使用`.ioc`文件生成工程(Stm32CubeIDE工具链)
4. 在`CMakeLists.txt`中添加/修改以下代码
- 去除注释，打开硬件浮点支持
```cmake
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
```
- 添加Include路径
```cmake
include_directories(
        xxx
        WTR_micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include
)
```
- 添加源文件
```cmake
file(GLOB_RECURSE SOURCES "Core/*.*"
        "Middlewares/*.*"
        "Drivers/*.*"
        "WTR_micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c"
        "WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c"
        "WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_time.c"
        "WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c"#按照你选择的传输方式添加对应的源文件
)
```
- 在`add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT}`后添加(加入库文件):
```cmake

# 构建绝对路径
set(LIBMICROROS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/WTR_micro_ros_stm32cubemx_utils/microros_static_library/libmicroros")
# 添加库文件搜索路径
link_directories(${LIBMICROROS_PATH})
# 链接库，使用库的实际名称
target_link_libraries(${PROJECT_NAME}.elf PRIVATE ${LIBMICROROS_PATH}/libmicroros.a)
```
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
ros2 trans_i2o micro_ros_setup create_agent_ws.sh
#构建代理包
ros2 trans_i2o micro_ros_setup build_agent.sh
source install/local_setup.bash

```
3. 运行micro-ROS 应用程序（串口）
```bash
ros2 trans_i2o micro_ros_agent micro_ros_agent serial --dev [device]
# eg:ros2 trans_i2o micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
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



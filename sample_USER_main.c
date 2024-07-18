#include "main.h"
#include "cmsis_os.h"
#include "USER_main.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
//include msg
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#ifdef __cplusplus
extern "C" {
#endif
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

#ifdef __cplusplus
}
#endif
//声明micro ros 组件
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
//声明  自定义组件
rcl_subscription_t servo_cmd_subscriber;
rcl_publisher_t debug_publisher;
rcl_publisher_t msgj_feedback_publisher;
rcl_publisher_t stm_time_publisher;
rcl_timer_t timer1;
//声明自定义消息
sensor_msgs__msg__JointState _msgj_in;
sensor_msgs__msg__JointState msgj_in;
std_msgs__msg__Float32MultiArray pos_feedback;
std_msgs__msg__Int32 stm_time;
std_msgs__msg__String debugmsg;

//声明函数
void servo_subscribe_callback(const void *msgin);

void joint_msg_init(sensor_msgs__msg__JointState *msg);

void joint_msg_fini(sensor_msgs__msg__JointState *msg);


//micro ros 线程 , LDR 常亮表示初始化错误
void UserStartDefaultTask(void *argument) {
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    if (rmw_uros_set_custom_transport(
            true,
            (void *) &huart8,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read) == RMW_RET_ERROR)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    // micro-ROS app
    allocator = rcl_get_default_allocator();
    //create support
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_support_init(&support, 0, NULL, &allocator)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    // create node
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_node_init_default(&node, "stm32_node", "", &support)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    std_msgs__msg__String__init(&debugmsg);

    // create publisher
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_publisher_init_default(
            &debug_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "debug")
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_publisher_init_default(
            &stm_time_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "stm32_time")
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
     //create subscriber
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_subscription_init_default(
            &servo_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "joint_cmd")
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //create timer
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if( rclc_timer_init_default(
            &timer1,
            &support,
            RCL_MS_TO_NS(100),
            timer1_callback)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //create executor
    executor = rclc_executor_get_zero_initialized_executor();
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_executor_init(&executor, &support.context, 2, &allocator)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //add subscriber and timer to executor
    rclc_executor_add_subscription(
            &executor,
            &servo_cmd_subscriber,
            &_msgj_in,
            &servo_subscribe_callback,
            ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &timer1);
    osDelay(300);
// 为数组型的msg分配空间
    joint_msg_init(&_msgj_in);
    joint_msg_init(&msgj_in);
    pos_feedback.data.capacity=3;
    pos_feedback.data.size=3;
    pos_feedback.data.data=(float *)pvPortMalloc(3*sizeof(float));
    xSemaphoreGive(sync_mutex);
    osDelay(10);
    xSemaphoreTake(sync_mutex, portMAX_DELAY);
    rclc_executor_spin(&executor);
    for (;;) {}
}
//定时器回调函数，用于发布消息
void timer1_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, GPIO_PIN_RESET);
    stm_time.data = (int32_t)xTaskGetTickCount();
    rcl_publish(&stm_time_publisher, &stm_time, NULL);
    /*if(xSemaphoreTake(data_mutex, 70/portTICK_RATE_MS)==pdTRUE)
    {
        rcl_publish(&msgj_feedback_publisher, &pos_feedback, NULL);
        xSemaphoreGive(data_mutex);
    }*/
    HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, GPIO_PIN_SET);
}
//JointState 只初始化能用到的字段
void joint_msg_init(sensor_msgs__msg__JointState *msg) {
    msg->header.frame_id.data = (char *) pvPortMalloc(3 * sizeof(char));
    msg->header.frame_id.capacity = 3;
    msg->name.data->data = (char *) pvPortMalloc(3 * sizeof(char));
    msg->name.data->capacity = 3;
    msg->position.data = (double *) pvPortMalloc(3 * sizeof(double));
    msg->position.capacity = 3;
    msg->velocity.data = (double *) pvPortMalloc(3 * sizeof(double));
    msg->velocity.capacity = 3;
    msg->effort.data = (double *) pvPortMalloc(3 * sizeof(double));
    msg->effort.capacity = 3;
}

//接收回调函数
void servo_subscribe_callback(const void *msgin) {
    const sensor_msgs__msg__JointState *_msgj_in = (const sensor_msgs__msg__JointState *) msgin;
    if (xSemaphoreTake(data_mutex, 10 / portTICK_RATE_MS) == pdTRUE) {
        sensor_msgs__msg__JointState__copy(_msgj_in, &msgj_in);
        xSemaphoreGive(data_mutex);
    } else {
        return;
    }
    /*发送dubug topic
    debugmsg.data.capacity = 20;
    debugmsg.data.size = 20;
    debugmsg.data.data = (char *) pvPortMalloc(20 * sizeof(char));
    sprintf(debugmsg.data.data, "%f", msgj_in.position.data[2]);
    rcl_publish(&debug_publisher, &debugmsg, NULL);
    vPortFree(debugmsg.data.data);*/

}


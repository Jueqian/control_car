#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include <fstream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x02
#define XBOX_BUTTON_Y 0x03
#define XBOX_BUTTON_LB 0x04
#define XBOX_BUTTON_RB 0x05
#define XBOX_BUTTON_START 0x06
#define XBOX_BUTTON_BACK 0x07
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x03 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x04 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x02
#define XBOX_AXIS_RT 0x05
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

// #define XBOX_AXIS_VAL_UP        -32767
// #define XBOX_AXIS_VAL_DOWN      32767
// #define XBOX_AXIS_VAL_LEFT      -32767
// #define XBOX_AXIS_VAL_RIGHT     32767

// #define XBOX_AXIS_VAL_MIN       -32767
// #define XBOX_AXIS_VAL_MAX       32767
// #define XBOX_AXIS_VAL_MID       0x00

typedef struct xbox_map
{
    int time;
    int a;
    int b;
    int x;
    int y;
    int lb;
    int rb;
    int start;
    int back;
    int home;
    int lo;
    int ro;

    int lx;
    int ly;
    int rx;
    int ry;
    int lt;
    int rt;
    int xx;
    int yy;

} xbox_map_t;

int xbox_open(const char *file_name)
{
    int xbox_fd;

    xbox_fd = open(file_name, O_RDONLY | O_NONBLOCK);
    // xbox_fd = open(file_name, O_RDONLY);
    if (xbox_fd < 0)
    {
        perror("open");
        return -1;
    }

    return xbox_fd;
}

int xbox_map_read(int xbox_fd, xbox_map_t *map)
{
    int len=0, type, number, value;
    struct js_event js;
    // ROS_INFO("now read the xbox");

    while (read(xbox_fd, &js, sizeof(struct js_event)) > 0)
    {
        // Do nothing, just read and discard the data
        len = sizeof(struct js_event);
        // ROS_INFO_STREAM("stop at the while and len is: " << len << "\n");
        // // ROS_INFO_STREAM("js type: " << js.type);
        // // ROS_INFO_STREAM("js number: " << js.number);
        // // ROS_INFO_STREAM("js value: " << js.value);
        // printf("js type: %d\t", js.type);
        // printf("js number: %d\t", js.number);
        // printf("js value: %d\n", js.value);

        // usleep(5 * 1000);   // @xxd: confuse, there must delay, or not the len always -1 when no data.(Normal is 0)

        type = js.type;
        number = js.number;
        value = js.value;

        map->time = js.time;

        if (type == JS_EVENT_BUTTON)
        {
            switch (number)
            {
            case XBOX_BUTTON_A:
                map->a = value;
                break;

            case XBOX_BUTTON_B:
                map->b = value;
                break;

            case XBOX_BUTTON_X:
                map->x = value;
                break;

            case XBOX_BUTTON_Y:
                map->y = value;
                break;

            case XBOX_BUTTON_LB:
                map->lb = value;
                break;

            case XBOX_BUTTON_RB:
                map->rb = value;
                break;

            case XBOX_BUTTON_START:
                map->start = value;
                break;

            case XBOX_BUTTON_BACK:
                map->back = value;
                break;

            case XBOX_BUTTON_HOME:
                map->home = value;
                break;

            case XBOX_BUTTON_LO:
                map->lo = value;
                break;

            case XBOX_BUTTON_RO:
                map->ro = value;
                break;

            default:
                break;
            }
        }
        else if (type == JS_EVENT_AXIS)
        {
            switch (number)
            {
            case XBOX_AXIS_LX:
                map->lx = value;
                break;

            case XBOX_AXIS_LY:
                map->ly = value;
                break;

            case XBOX_AXIS_RX:
                map->rx = value;
                break;

            case XBOX_AXIS_RY:
                map->ry = value;
                break;

            case XBOX_AXIS_LT:
                map->lt = value;
                break;

            case XBOX_AXIS_RT:
                map->rt = value;
                break;

            case XBOX_AXIS_XX:
                map->xx = value;
                break;

            case XBOX_AXIS_YY:
                map->yy = value;
                break;

            default:
                break;
            }
        }
        else
        {
            /* Init do nothing */
        }
    }
    // len = read(xbox_fd, &js, sizeof(struct js_event));
    if (len < 0)
    {
        // perror("read");
        ROS_INFO("len is -1");
        return -1;
    }

    return len;
}

void xbox_close(int xbox_fd)
{
    close(xbox_fd);
    return;
}

/*
按照下面对应关系调整按键的对应关系
// 对于我的自己的手柄，键位对应如下：
printf("\r(junjie)A:%d B:%d X:%d Y:%d LB:%d RB:%d start(0):%d back:%d home(0):%d LO(0):%d RO(0):%d XX:%-6d YY:%-6d
LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
map.a, map.b, map.y, map.lb, map.start, map.back, map.start, map.ro, map.home, map.lo, map.ro, map.xx, map.yy,
map.lx, map.ly, map.lt, map.rx, map.rt, map.ry);

注意这里：
LT和RT初始位置是-32767 最下面是32767
LY下是32767，LX右边是32767
RY下是32767，RX右边是32767
*/
void transform_map(xbox_map_t &new_map, xbox_map_t old_map)
{
    new_map.a = old_map.a;
    new_map.b = old_map.b;
    new_map.x = old_map.y;
    new_map.y = old_map.lb;
    new_map.lb = old_map.start;
    new_map.rb = old_map.back;
    new_map.start = old_map.start;
    new_map.back = old_map.ro;
    new_map.home = old_map.home;
    new_map.lo = old_map.lo;
    new_map.ro = old_map.ro;
    new_map.xx = old_map.xx;
    new_map.yy = old_map.yy;
    new_map.lx = old_map.lx;
    new_map.ly = old_map.ly;
    new_map.rx = old_map.lt;
    new_map.ry = old_map.rx;
    new_map.lt = old_map.rt;
    new_map.rt = old_map.ry;
}

bool isFileExists_ifstream(std::string &name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_simulate");
    ros::NodeHandle nh;

    int xbox_fd;
    xbox_map_t map;
    xbox_map_t my_map;
    int len, type;
    int axis_value, button_value;
    int number_of_axis, number_of_buttons;

    memset(&map, 0, sizeof(xbox_map_t));
    std::string xbox_path;
    double vel_x_scale, ang_z_scale;
    nh.getParam("xbox_path", xbox_path);
    nh.getParam("vel_x_scale", vel_x_scale);
    nh.getParam("ang_z_scale", ang_z_scale);

    xbox_fd = xbox_open(xbox_path.c_str());
    if (xbox_fd < 0)
    {
        std::cout << "can not open the xbox! " << std::endl;
        return -1;
    }
    len = xbox_map_read(xbox_fd, &map); // @xxd: clear the buffer

    ros::Publisher cmd_vel_sim = nh.advertise<geometry_msgs::Twist>("/car/cmd_vel", 5);
    ros::Rate rate(20);
    bool status = ros::ok();
    geometry_msgs::Twist velocity_cmd;

    // 先初始化一下
    velocity_cmd.linear.x = 0.0;
    velocity_cmd.linear.y = 0.0;
    velocity_cmd.linear.z = 0.0;
    velocity_cmd.angular.x = 0.0;
    velocity_cmd.angular.y = 0.0;
    velocity_cmd.angular.z = 0.0;

    ROS_INFO_STREAM("vel_x_scale : " << vel_x_scale << "  ang_z_scale : " << ang_z_scale);
    ROS_INFO_STREAM("linear.x : " << velocity_cmd.linear.x << " angular.z : " << velocity_cmd.angular.z << "\n");

    while (status)
    {
        // 判断手柄是否意外退出
        if (!isFileExists_ifstream(xbox_path))
        {
            ROS_WARN("xbox controller is not connected !!!! ");
            velocity_cmd.linear.x = 0.0;
            velocity_cmd.angular.z = 0.0;
            cmd_vel_sim.publish(velocity_cmd); // 发布，让车停止
            return -1;
        }

        len = xbox_map_read(xbox_fd, &map);
        // std::cout << len << "-----------------------" << std::endl;

        if (len < 0)
        {
            // usleep(1 * 1000);
            std::cout << "Something maybe error, the len is:" << len << std::endl;
            continue;
        }
        else if (len > 0)
        {
            transform_map(my_map, map);

            // 将按键设置为速度（这里使用ly来控制前后车速，使用rx来控制转向速度）
            // angular .z 左转为正，右转为负(根据右手定则，正的角速度表示逆时针旋转，而负的角速度表示顺时针旋转)
            double vel_x = (double)((double)-my_map.ly / 32767.0);
            double vel_ang = (double)((double)-my_map.rx / 32767.0);

            if (my_map.x == 1)
                ang_z_scale /= 2;
            if (my_map.b == 1)
                ang_z_scale += 2;
            if (my_map.y == 1)
                vel_x_scale += 2;
            if (my_map.a == 1)
                vel_x_scale /= 2;

            velocity_cmd.linear.x = vel_x * vel_x_scale;
            velocity_cmd.angular.z = vel_ang * ang_z_scale;

            ROS_INFO_STREAM("vel_x_scale : " << vel_x_scale << "  ang_z_scale : " << ang_z_scale);
            ROS_INFO_STREAM("linear.x : " << velocity_cmd.linear.x << " angular.z : " << velocity_cmd.angular.z << "\n");

            cmd_vel_sim.publish(velocity_cmd); // 发布

            // ROS_INFO("velocity_cmd linear.x: %f, linear.y: %f, linear.z: %f, angular.x: %f, angular.y: %f, angular.z: %f",
            //          velocity_cmd.linear.x, velocity_cmd.linear.y, velocity_cmd.linear.z,
            //          velocity_cmd.angular.x, velocity_cmd.angular.y, velocity_cmd.angular.z);
        }

        

        fflush(stdout);
        status = ros::ok();
        rate.sleep();
    }

    xbox_close(xbox_fd);
    return 0;
}

// // 对于我的自己的手柄，键位对应如下：
// printf("\r(junjie)A:%d B:%d X:%d Y:%d LB:%d RB:%d start(0):%d back:%d home(0):%d LO(0):%d RO(0):%d
// XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
//         map.a, map.b, map.y, map.lb, map.start, map.back, map.start, map.ro, map.home, map.lo, map.ro,
//         map.xx, map.yy, map.lx, map.ly, map.lt, map.rx, map.rt, map.ry);

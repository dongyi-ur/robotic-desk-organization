/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "serial_msgs/serial.h"

#define rBUFFERSIZE 8
unsigned char r_buffer[rBUFFERSIZE];
serial::Serial ser;
std_msgs::String gripper;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    gripper.data = msg->data;
}

/*std::vector<unsigned char> cov(unsigned char* rbu)
                         {
                           std::vector<unsigned char>  m;
                           for(int l=0;l<rBUFFERSIZE;l++)
                                      {
                                      m.push_back(rbu[l]);
                                       }
                             return m;
                           }*/

int main (int argc, char** argv)
{
    ros::init(argc, argv, "serial_example_node1");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("/gripper_control", 10, write_callback);
    ros::Publisher msg_pub = nh.advertise<serial_msgs::serial>("read1", 10);

    //////////////////////////////////////////////////////////////////////////////
    ros::Rate loop_rate(50);

    while(ros::ok()){

        serial_msgs::serial msg;
        msg.serial.clear();

        ros::spinOnce();

        if(gripper.data == "close")
        {
            r_buffer[0] = 0x01;
            r_buffer[1] = 0x05;
            r_buffer[2] = 0x01;
            r_buffer[3] = 0x00;
            r_buffer[4] = 0xFF;
            r_buffer[5] = 0x00;
            r_buffer[6] = 0x8D;
            r_buffer[7] = 0xC6;
        }
        else if(gripper.data  == "relax_close")
        {
            r_buffer[0] = 0x01;
            r_buffer[1] = 0x05;
            r_buffer[2] = 0x01;
            r_buffer[3] = 0x00;
            r_buffer[4] = 0x00;
            r_buffer[5] = 0x00;
            r_buffer[6] = 0xCC;
            r_buffer[7] = 0x36;
        }
        else if(gripper.data  == "open")
        {
            r_buffer[0] = 0x01;
            r_buffer[1] = 0x05;
            r_buffer[2] = 0x01;
            r_buffer[3] = 0x01;
            r_buffer[4] = 0xFF;
            r_buffer[5] = 0x00;
            r_buffer[6] = 0xDC;
            r_buffer[7] = 0x06;
        }
        else if(gripper.data == "relax_open")
        {
            r_buffer[0] = 0x01;
            r_buffer[1] = 0x05;
            r_buffer[2] = 0x01;
            r_buffer[3] = 0x01;
            r_buffer[4] = 0x00;
            r_buffer[5] = 0x00;
            r_buffer[6] = 0x9D;
            r_buffer[7] = 0xF6;
        }

        // for(int i=0;i<rBUFFERSIZE;i++)
        // {
        //     ROS_INFO("[0x%02x]",r_buffer[i]);
        // }

        for(int l=0;l<rBUFFERSIZE;l++)
        {
            msg.serial.push_back(r_buffer[l]);
        }

        msg_pub.publish(msg);
 
        loop_rate.sleep();

    }
    //////////////////////////////////////////////////////////////////////////////

    /**/
    // try
    // {
    //     ser.setPort("/dev/ttyUSB1");
    //     ser.setBaudrate(9600);
    //     serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //     ser.setTimeout(to);
    //     ser.open();
    // }
    // catch (serial::IOException& e)
    // {
    //     ROS_ERROR_STREAM("Unable to open port ");
    //     return -1;
    // }

    // if(ser.isOpen()){
    //     ROS_INFO_STREAM("Serial Port initialized");
    // }else{
    //     return -1;
    // }

    // ros::Rate loop_rate(50);

    // while(ros::ok()){

    //     serial_msgs::serial msg;

    //     ros::spinOnce();

    //     if(ser.available())
    //     {
    //         ROS_INFO_STREAM("Reading from serial port");

    //         ser.read(r_buffer,rBUFFERSIZE);

	// 		for(int i=0;i<rBUFFERSIZE;i++)
    //         {
    //             ROS_INFO("[0x%02x]",r_buffer[i]);
    //         }
            
	// 		ROS_INFO_STREAM("End reading from serial port");  

    //         for(int l=0;l<rBUFFERSIZE;l++)
    //         {
    //             msg.serial.push_back(r_buffer[l]);
    //         }

    //         msg_pub.publish(msg);

    //         /*  
    //         for(int j=0;j<rBUFFERSIZE;j++)
    //         ROS_INFO("[0x%02x]",msg.serial[j]); */   
                              
    //         /*   
    //         std::cout << "msg.serial[0]=" << msg.serial[j] << std::endl; 
    //         std::cout << "msg.serial.size=" << msg.serial.size() << std::endl;
    //         //std_msgs::String s( reinterpret_cast<char const*>(r_buffer) );
    //         ser.write(r_buffer,rBUFFERSIZE); */   
    //     } 

    //     /* 此处代码为读取字符串类型
    //     if(ser.available())
    //     {
    //         ROS_INFO_STREAM("Reading from serial port");
    //         std_msgs::String result;
    //         result.data=ser.read(ser.available());
    //         ROS_INFO_STREAM("Read: " << result.data);
    //         read_pub.publish(result);
    //     } */

    //     loop_rate.sleep();

    // }
}


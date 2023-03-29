#include <ros/ros.h>
#include <iostream>

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

void scan_to_cloud(int ww, int hh, pcl::PointCloud<Point>& cloud) {
    // cloud.resize(ls.w * ls.h);
    // auto points = ouster::cartesian(ls, xyz_lut);

    for (auto height = 0; height < hh; height++) {
        for (auto width = 0; width < ww; width++) {
            // const auto xyz = points.row(u * ls.w + v);
            // const auto pix = ls.data.row(u * ls.w + v);
            // const auto ts = (ls.header(v).timestamp - scan_ts).count();

            // {x, y, z, 1.0}, Intensity, ts, Reflectivity, u, Ambient, Range
            cloud(width, height) = Point{
                {{static_cast<float>(height), static_cast<float>(width),
                  static_cast<float>(height * width * 0.2), 1.0f}},
                static_cast<float>(2),
                static_cast<uint32_t>(3),
                static_cast<uint16_t>(4),
                static_cast<uint8_t>(5),
                static_cast<uint16_t>(6),
                static_cast<uint32_t>(7)};
        }
    }
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(int ww, int hh, const pcl::PointCloud<Point>& cloud, int timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg{};
    pcl::toROSMsg(cloud, msg);
 
    // *** header->seq *** automated setting
    // *** header->stamp setting *** If you set it(nanosec), (stamp.sec, stamp.nsec) be set.
    msg.header.stamp.fromNSec(10000000000);
    msg.header.stamp = ros::Time::now();

    /// ***header->frame_id setting***
    msg.header.frame_id = frame;
    
    return msg;
}

int main(int argc, char** argv)
{
    const double PI = 3.1415926;

    time_t start, end;;

    ros::init(argc, argv, "carnavi_cloud_node");
    ros::Publisher lidar_pub;
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_points", 1);

    ///// udp /////

    int sock = socket(AF_INET, SOCK_DGRAM, 0); //(주소 체계, 소켓 타입, 프로토콜)
    int retval;

    sockaddr_in serveraddr;
    bzero(&serveraddr, sizeof(serveraddr)); //0으로 초기화

    serveraddr.sin_family=AF_INET;
    serveraddr.sin_port=htons(5000);
    serveraddr.sin_addr.s_addr=htonl(INADDR_ANY);

    bind(sock, (sockaddr*)&serveraddr, sizeof(serveraddr));

    struct ip_mreq mreq;
    char* group = "224.0.0.5";
    mreq.imr_multiaddr.s_addr = inet_addr(group);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (
        setsockopt(
            sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
        ) < 0
    ){
        perror("setsockopt");
        return 1;
    }
     
    // 데이터 통신에 사용할 변수
    sockaddr_in clientaddr;
    int addrlen;
    uint8_t buf[1024];

    // ***width***
    int W = 480;
    // // ***height***
    int H = 2;
    pcl::PointCloud<Point> cloud{W, H};

    //int vfov = -3;
   
    // 클라이언트와 데이터 통신
    while (ros::ok()){
        start = time(NULL);

        addrlen=sizeof(clientaddr);
        // retval=recvfrom(sock,buf,1024,0,(sockaddr*)&clientaddr, (socklen_t*)&addrlen);
        retval=recvfrom(sock,buf,1024,0,(sockaddr*)&clientaddr, (socklen_t*)&addrlen);
        buf[retval]='\0';

        //calculate the coordinates

        for(int v = 0; v < 1; v++){
            for(int h = 7; h < 487; h++){
                int v_ch = v + 1;
                int h_ch = h - 6;
                int real_v_ind = v;
                int real_h_ind = h + h_ch;
                // double angle_v = 3 + real_v_ind * vfov;
                double angle_h = (120.0/479.0)*h_ch - (28860.0/479.0);
                double distance = 0.01 * (buf[real_h_ind]+100*buf[real_h_ind-1]);
                double channel = 0;

                if(buf[4] == 0xc1){
                    channel = 1;
                }

                double z = distance * sin(3 * channel * PI/180.0);
                double diag_d = distance * cos(3 * channel * PI/180.0);
                double x = diag_d * cos(angle_h * PI / 180.0);
                double y = diag_d * sin(angle_h * PI / 180.0);

                //test should be removed
                // x = h % 20;
                // y = h / 20;
                // if(buf[4] == 0xc1){
                //     z = 5;
                // }

                //fill the point data
                cloud(h_ch-1, channel) =  Point{
                {{static_cast<float>(x), static_cast<float>(y),
                  static_cast<float>(z), 1.0f}},
                static_cast<float>(2),
                static_cast<uint32_t>(3),
                static_cast<uint16_t>(4),
                static_cast<uint8_t>(channel),
                static_cast<uint16_t>(6),
                static_cast<uint32_t>(7)};
            }
        }

        if(buf[4] == 0xc1){
            lidar_pub.publish(cloud_to_cloud_msg(W, H, cloud, 100, "sensor_frame"));
        }
        
        // ros::spin();
        ros::spinOnce();
        loop_rate.sleep();
        printf("time = %f\n", (double)(time(NULL)-start));
    }

    close(sock);

    return 0;
} 
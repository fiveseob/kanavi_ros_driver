#include "sample.h"

#define W 1200  // HFoV, 1ch = 1200, 2ch = 480
#define H 1  // VFoV, 1ch = 1, 2ch = 2
#define BUF_SIZE 4096
#define PORT_NUM 5000
#define MULTI_IP "224.0.0.5"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "carnavi_cloud_node");
    ros::Publisher lidar_pub; 
    ros::NodeHandle nh;
    ros::Rate loop_rate(30); 

    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_points",1);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    int retval;

    sockaddr_in serveraddr;
    bzero(&serveraddr, sizeof(serveraddr));

    serveraddr.sin_family=AF_INET;
    serveraddr.sin_port=htons(PORT_NUM);
    serveraddr.sin_addr.s_addr=htonl(INADDR_ANY);

    if(bind(sock, (sockaddr*)&serveraddr, sizeof(serveraddr))==-1)
        cout << "bind error" << endl;

    struct ip_mreq mreq;
    char *group = MULTI_IP;

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

    sockaddr_in clientaddr;
    int addrlen;
    
    uint8_t fst_buf[BUF_SIZE];
    uint8_t temp[BUF_SIZE];
    uint8_t buf[BUF_SIZE];

    pcl::PointCloud<Point> cloud{W,H};

    while (ros::ok()){

        clock_t start,end;
        double result;

        start = clock();

        addrlen = sizeof(clientaddr);

        retval = recvfrom(sock, fst_buf, BUF_SIZE, 0, (sockaddr*)&clientaddr, (socklen_t*)&addrlen);
        fst_buf[retval]='\0';

        if(retval == 1472){
            memcpy(temp,fst_buf,sizeof(fst_buf));
        }
        else if(retval == 937){
            for(int i=0;i<935;i++){
                temp[1472+i] = fst_buf[i];
                memcpy(buf,temp,sizeof(temp));

            }
        }

        // calculate the coordinates
        for(int h=7; h < 1207; h++){
            int h_ch = h - 6;
            int real_h_ind = h + h_ch;
            
            double angle_h = (300.0/1199.0)*h_ch - (180150.0/1199.0);
            double distance =  0.01*(buf[real_h_ind]+100*buf[real_h_ind - 1]);
            double channel = 0;
    
            double x = distance * cos( angle_h * M_PI / 180.0);
            double y = distance * sin( angle_h * M_PI / 180.0);
        
            cloud(h_ch-1, channel) = Point{
            {{static_cast<float>(x), static_cast<float>(y),
              static_cast<float>(0), 1.0f}},
            static_cast<float>(2),
            static_cast<uint32_t>(3),
            static_cast<uint16_t>(4),
            static_cast<uint8_t>(5),
            static_cast<uint16_t>(6),
            static_cast<uint32_t>(7)};  
        }

        if(buf[4] == 0xc0){
            lidar_pub.publish(cloud_to_cloud_msg(W,H,cloud,100,"sensor_frame"));
        }

        memset(buf,0,sizeof(buf));
        memset(fst_buf,0,sizeof(fst_buf));

        ros::spinOnce();
        loop_rate.sleep();
        
        end = clock();
        result = (double)(end - start);

        cout << "Processing Time : "<< ((result)/CLOCKS_PER_SEC)<<" seconds" << endl;
        cout << "Processing Time : "<< result << " microseconds" << endl;
    }

    close(sock);

    return 0;

}




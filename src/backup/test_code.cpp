#include "sample.h"

#define W 480    // h방향 point 개수.
#define H 2    // 채널 개수.
#define DIS_TO_WALL 3.0
#define BUF_SIZE 1024
#define PORT_NUM 5000
#define MULTI_IP "224.0.0.5"

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "carnavi_cloud_node");   //노드 초기화 및 이름 지정
    ros::Publisher lidar_pub;
	ros::Publisher lidar_pub_gt;
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);

    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_points",1); // 메세지타입 및 메세지명? 
	lidar_pub_gt = nh.advertise<sensor_msgs::PointCloud2>("/test_points_gt",1); // 메세지타입 및 메세지명? 

// 아래 부분은 socket 통신(UDP) 관련
	int sock = socket(AF_INET, SOCK_DGRAM, 0);    // 소켓 생성
	int retval;

	sockaddr_in serveraddr; //수켓 주소 설정
	bzero(&serveraddr, sizeof(serveraddr));

	// bind 준비 
	serveraddr.sin_family=AF_INET;
    serveraddr.sin_port=htons(PORT_NUM);
    serveraddr.sin_addr.s_addr=htonl(INADDR_ANY);   // INADDR_ANY : 컴퓨터에 존재하는 랜카드 중에서 사용 가능한 IP 주소를 찾아서 자동으로 대입함

    bind(sock, (sockaddr*)&serveraddr, sizeof(serveraddr));

    struct ip_mreq mreq;
    char *group = MULTI_IP;

    mreq.imr_multiaddr.s_addr = inet_addr(group);     // 가입할 멀티캐스트 주소
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);     // 멀티캐스트 패킷을 받을 네트워크 인터페이스

    if (
        setsockopt(
            sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)    // 생성된 socket에 속성값을 변경하는 함수 
        ) < 0
    ){
        perror("setsockopt");
        return 1;
    }

    sockaddr_in clientaddr;
    int addrlen;
    uint8_t buf[BUF_SIZE];

    pcl::PointCloud<Point> cloud{W,H}; // pcl 라이브러리 메세지? 생성
    pcl::PointCloud<Point> cloud_gt{W,H}; // pcl 라이브러리 메세지? 생성

    int error_count, count = 0;
    double sum, error_dist= 0;

    while (ros::ok()){
        
        clock_t start,end;
        double result;

        start = clock();

        addrlen = sizeof(clientaddr);
        retval = recvfrom(sock, buf, BUF_SIZE, 0, (sockaddr*)&clientaddr, (socklen_t*)&addrlen);
        buf[retval]='\0';

        // calculate the coordinates
        for(int v=0; v < 1; v++){
            for(int h=7; h < 487; h++){
                int v_ch = v + 1;
                int h_ch = h - 6;
                int real_v_ind = v;
                int real_h_ind = h + h_ch;
                
                double angle_h = (120.0/479.0)*h_ch - (28860.0/479.0);
                double distance =  0.01*(buf[real_h_ind]+100*buf[real_h_ind - 1]);
                double channel = 0;
        
                if(buf[4] == 0xc1){
                    channel = 1;
                }   
                
                // predict        
                double diag_d = distance * cos(3 * channel * M_PI / 180.0);
                double x = diag_d * cos( angle_h * M_PI / 180.0);
                double y = diag_d * sin( angle_h * M_PI / 180.0);
                // double z = distance * sin( 3 * channel * M_PI / 180.0);
                double z = x * sin( 3 * channel * M_PI / 180.0);
                
                // double real_dis = sqrt(x^2 + y^2 + z^2)
                double pred_dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

                cloud(h_ch-1, channel) = Point{
                {{static_cast<float>(x), static_cast<float>(y),
                  static_cast<float>(z), 1.0f}},
                static_cast<float>(2),
                static_cast<uint32_t>(3),
                static_cast<uint16_t>(4),
                static_cast<uint8_t>(5),
                static_cast<uint16_t>(6),
                static_cast<uint32_t>(7)};

                //gt
                double x_gt = (double)DIS_TO_WALL;
                double y_gt = (double)DIS_TO_WALL * tan(angle_h * M_PI / 180.0);

                double dist_gt = (double)DIS_TO_WALL / cos(angle_h * M_PI / 180.0);

                // double z = tan(3 * (double)channel * M_PI / 180.0) * dist;
                double z_gt = tan(3 * (double)channel * M_PI / 180.0) * DIS_TO_WALL;
                double gt_dist = sqrt(pow(x_gt, 2) + pow(y_gt, 2) + pow(z_gt, 2));

                cloud_gt(h_ch-1, channel) = Point{
                {{static_cast<float>(x_gt), static_cast<float>(y_gt),
                  static_cast<float>(z_gt), 1.0f}},
                static_cast<float>(2),
                static_cast<uint32_t>(3),
                static_cast<uint16_t>(4),
                static_cast<uint8_t>(5),
                static_cast<uint16_t>(6),
                static_cast<uint32_t>(7)};  


                //error calculate
                // error_dist = (gt_dist - pred_dist) / gt_dist;
                // error_count += 1;

                // sum = sum + error_dist;

                // if((error_count % 10000) == 0){
                //     cout << error_count << " points were collected." << endl;
                // }
                // if((error_count % 1000000) == 0)
                // {
                //     cout << "av_error = " << 100*(sum  / 1000000) << endl;

                //     error_count = 0;
                //     sum = 0;    
                // }
                
                if(h >= 67 && h < 427){
                    count += 1;

                    if ((gt_dist - pred_dist) > 0.20)
                        error_count += 1;

                    if((count % 10000) == 0){
                        cout << count << " points were collected." << endl;
                    }
                    
                    if((count % 1000000) == 0)
                    {
                        cout << "av_error = " << 100*(error_count / 1000000.0) << endl;

                        count = 0;
                           
                    }
                }
                
                
            }
        }

        if(buf[4] == 0xc1){
            lidar_pub.publish(cloud_to_cloud_msg(W,H,cloud,100,"sensor_frame"));
            lidar_pub_gt.publish(cloud_to_cloud_msg(W,H,cloud_gt,100,"sensor_frame"));
        }


        ////making gt

        for(int v=0; v < 1; v++){
            for(int h=7; h < 487; h++){
                int v_ch = v + 1;
                int h_ch = h - 6;
                int real_v_ind = v;
                int real_h_ind = h + h_ch;
                
                double angle_h = (120.0/479.0)*h_ch - (28860.0/479.0);
                double channel = 0;
        
                if(buf[4] == 0xc1){
                    channel = 1;
                }   
                double x = (double)DIS_TO_WALL;
                double y = (double)DIS_TO_WALL * tan(angle_h * M_PI / 180.0);

                double dist = (double)DIS_TO_WALL / cos(angle_h * M_PI / 180.0);

                // double z = tan(3 * (double)channel * M_PI / 180.0) * dist;
                double z = tan(3 * (double)channel * M_PI / 180.0) * DIS_TO_WALL;

                cloud_gt(h_ch-1, channel) = Point{
                {{static_cast<float>(x), static_cast<float>(y),
                  static_cast<float>(z), 1.0f}},
                static_cast<float>(2),
                static_cast<uint32_t>(3),
                static_cast<uint16_t>(4),
                static_cast<uint8_t>(5),
                static_cast<uint16_t>(6),
                static_cast<uint32_t>(7)};  
            }
        }

        if(buf[4] == 0xc1){
        }

        ////making gt end

        ros::spinOnce();
        loop_rate.sleep();
        
        end = clock();
        result = (double)(end - start);

        // cout << "Processing Time : "<< ((result)/CLOCKS_PER_SEC) * 1000.0<<" ms" << endl;
        // cout << "Processing Time : "<< result << " microseconds" << endl;
    }

    close(sock);

    return 0;

}


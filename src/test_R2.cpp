#include "sample.h"

#define W 480  
#define H 2    
#define BUF_SIZE 1024
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

    bind(sock, (sockaddr*)&serveraddr, sizeof(serveraddr));

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
    uint8_t buf[BUF_SIZE];
//    unsigned int buf[BUF_SIZE];

    pcl::PointCloud<Point> cloud{W,H};

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
    			double z = distance * sin( 3 * channel * M_PI / 180.0);
    			double diag_d = distance * cos(3 * channel * M_PI / 180.0);
    			double x = diag_d * cos( angle_h * M_PI / 180.0);
    			double y = diag_d * sin( angle_h * M_PI / 180.0);
    		
    			cloud(h_ch-1, channel) = Point{
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
    		lidar_pub.publish(cloud_to_cloud_msg(W,H,cloud,100,"sensor_frame"));
    	}

        printf("----------------------------------------------------------\n");
        printf("Header buf[0] : %#x\n", buf[0]);
        printf("ProductLine buf[1] : %#x\n", buf[1]);
        printf("ID buf[2] : %#x\n", buf[2]);
        printf("Command_Mode buf[3] : %#x\n", buf[3]);
        printf("Command_Channel buf[4] : %#x\n", buf[4]);
        printf("DataLength buf[5] : %#x\n", buf[5]);
        printf("DataLength buf[6] : %#x\n", buf[6]);
        printf("Distance_Data_Int buf[7] : %#x\n", buf[7]);
        printf("Distance_Data_Float buf[8] : %#x\n", buf[8]);
        printf("buf[9] : %#x\n", buf[9]);
        printf("buf[10] : %#x\n", buf[10]);
        printf("buf[965] : %#x\n", buf[965]);
        printf("buf[966] : %#x\n", buf[966]);
        printf("buf[967] : %#x\n", buf[967]);
        printf("CheckSum buf[968] : %#x\n", buf[968]);

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




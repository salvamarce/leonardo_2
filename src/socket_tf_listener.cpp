
#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//Socket functions
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>

#include "boost/thread.hpp"

using namespace std;



struct Pose3D {
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
};

struct Poses3D {
	Pose3D bl_a;
	Pose3D o_a;
	Pose3D bl_o;
};


inline bool listener_socket(int port_number, int *sock) {
    sockaddr_in si_me;

    if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        std::cout << "Listener::Open: error during socket creation!" << std::endl;
        return false;
    }

    memset((char *) &si_me, 0, sizeof(si_me));

    /* allow connections to any address port */
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port_number);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

    if ( bind_ok == -1 )
        return false;
    else 
        return true;

}

class sock_tf_listener {
    public:
        sock_tf_listener();
        void run();
        void listener();

    private:
        ros::NodeHandle _nh;

};



sock_tf_listener::sock_tf_listener() {

}

void sock_tf_listener::listener() {

    int sock;
    listener_socket( 9090, &sock);

    Poses3D tf_poses;  
    int slen, rlen;
    sockaddr_in si_me, si_other;


    tf::Transform transform_bl_a;
    tf::Transform transform_o_a;
    tf::Transform transform_bl_o;

    tf::TransformBroadcaster broadcaster;
    while(ros::ok() ) {

        rlen = recvfrom(sock, &tf_poses, sizeof(tf_poses),0,(struct sockaddr*)&si_other, (socklen_t*)&slen);

        transform_bl_a.setOrigin(tf::Vector3( tf_poses.bl_a.x, tf_poses.bl_a.y, tf_poses.bl_a.z  )  );
        tf::Quaternion q_bl_a( tf_poses.bl_a.qx,  tf_poses.bl_a.qy,  tf_poses.bl_a.qz,  tf_poses.bl_a.qw);
        transform_bl_a.setRotation(q_bl_a);
        tf::StampedTransform stamp_transform_bl_a(transform_bl_a, ros::Time::now(), "arena", "base_link");


        transform_o_a.setOrigin(tf::Vector3( tf_poses.o_a.x, tf_poses.o_a.y, tf_poses.o_a.z  )  );
        tf::Quaternion q_o_a( tf_poses.o_a.qx,  tf_poses.o_a.qy,  tf_poses.o_a.qz,  tf_poses.o_a.qw);
        transform_o_a.setRotation(q_o_a);
        tf::StampedTransform stamp_transform_o_a(transform_o_a, ros::Time::now(), "arena", "odom");


        transform_bl_o.setOrigin(tf::Vector3( tf_poses.bl_o.x, tf_poses.bl_o.y, tf_poses.bl_o.z  )  );
        tf::Quaternion q_bl_o( tf_poses.bl_o.qx,  tf_poses.bl_o.qy,  tf_poses.bl_o.qz,  tf_poses.bl_o.qw);
        transform_bl_o.setRotation(q_bl_o);
        tf::StampedTransform stamp_transform_bl_o(transform_bl_o, ros::Time::now(), "odom", "base_link");

        broadcaster.sendTransform(stamp_transform_bl_a);
        broadcaster.sendTransform(stamp_transform_o_a);
        broadcaster.sendTransform(stamp_transform_bl_o);
  
   }
}



void sock_tf_listener::run() {

    boost::thread listener_t(&sock_tf_listener::listener, this);
    ros::spin();
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "sock_tf_listener");

    sock_tf_listener tf_l;
    tf_l.run();


    return 0;

}
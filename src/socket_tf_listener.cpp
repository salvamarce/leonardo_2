
#include "ros/ros.h"

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

    Poses3D p;  
    int slen, rlen;
    sockaddr_in si_me, si_other;

    while(ros::ok() ) {

        rlen = recvfrom(sock, &p, sizeof(p),0,(struct sockaddr*)&si_other, (socklen_t*)&slen);
        cout << "rlen: " << rlen << endl;

    }
}



void sock_tf_listener::run() {


    ros::spin();
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "sock_tf_listener");

    sock_tf_listener tf_l;
    tf_l.run();


    return 0;

}
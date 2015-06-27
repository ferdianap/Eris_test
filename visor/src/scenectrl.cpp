#include <ros/ros.h>
#include <cstdlib>
#include <visor/call.h>
#include <ctype.h>

#include <unistd.h>
#include <termios.h>

char getch() {
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		perror ("tcsetattr ~ICANON");
	return (buf);
}


int main(int argc, char** argv) {
 	ros::init(argc, argv, "scenectrl");
	ros::NodeHandle n;
	ros::ServiceClient client;
	std::string SERVER("");
	visor::call srv;
	int count=0;

	std::cout << "Press c to cycle between input scenes\n"
		"Press SPACE to publish topic /processImgNOW\n"
		"Press any other keys to quit\n"
		"============================================\n";

	try { // DO SOMETHING HERE !
	} catch (std::exception& e) {
		std::cerr << "Unhandled Exception reached the top of main: "
			<< e.what() << ", application will now exit\n";
    	return 1;
	}

	while (ros::ok()) {
		int input = getch();
		switch (input) {
		case 32: // Spacebar
			SERVER = "vattn_server";
			break;
		case 99: // c
			SERVER = "scenechange_server";
			break;
		default:
			exit(EXIT_SUCCESS);
			break;
		}
		// call service to process img now to node2 ServiceCallback
		client = n.serviceClient<visor::call>(SERVER);
		if (!client.call(srv)) {
			ROS_ERROR_STREAM("Failed to call service " << SERVER );
			return 2;
		} else {
			if (srv.response.resp.empty()) {
				printf("No results found.\n");
				return 0;
			}
			std::cout << "Service Called. <<"<< srv.response.resp <<"\n";
		}
	}
	return 0;
}

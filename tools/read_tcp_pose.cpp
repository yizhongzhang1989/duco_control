// Read the Duco controller's TCP pose directly via its Thrift RPC SDK.
//
// This talks to the robot controller on port 7003 using the precompiled
// libDucoCobotAPI.a, the same SDK the ROS driver uses. The values returned
// are exactly what the teach pendant shows (TCP pose in the base frame).
//
// Build (see tools/build_read_tcp_pose.sh) and run:
//   ./read_tcp_pose [robot_ip]   (default 192.168.1.10)

#include "duco_ros_driver/DucoCobot.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace DucoRPC;

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

void print_pose(const std::string& label, const std::vector<double>& p) {
  if (p.size() < 6) {
    std::cout << label << ": <no data> (size=" << p.size() << ")\n";
    return;
  }
  std::cout << std::fixed << std::setprecision(2);
  std::cout << label << ":\n"
            << "  x, y, z  (mm) : " << p[0] * 1000.0 << ", " << p[1] * 1000.0
            << ", " << p[2] * 1000.0 << "\n"
            << "  rx,ry,rz (deg): " << p[3] * kRadToDeg << ", "
            << p[4] * kRadToDeg << ", " << p[5] * kRadToDeg << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string ip = "192.168.1.10";
  bool csv = false;  // machine-readable: q1..q6,x,y,z,rx,ry,rz (rad,m,rad)
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--csv") {
      csv = true;
    } else {
      ip = a;
    }
  }
  const unsigned int port = 7003;

  DucoCobot robot(ip, port);
  if (robot.open() != 0) {
    std::cerr << "ERROR: could not connect to Duco controller at " << ip << ":"
              << port << "\n";
    return 1;
  }

  std::vector<double> q;
  std::vector<double> tcp;
  std::vector<double> flange;
  robot.get_actual_joints_position(q);  // axis order 1..6 (rad)
  robot.get_tcp_pose(tcp);        // tool end point in base frame (pendant TCP)
  robot.get_flange_pose(flange);  // link_6 flange in base frame

  if (csv) {
    // One line: q1..q6,x,y,z,rx,ry,rz  (joints rad, pos m, rot rad)
    std::cout << std::fixed << std::setprecision(8);
    for (size_t i = 0; i < q.size(); ++i) std::cout << q[i] << ",";
    for (size_t i = 0; i < tcp.size(); ++i) {
      std::cout << tcp[i] << (i + 1 < tcp.size() ? "," : "\n");
    }
    robot.close();
    return 0;
  }

  std::cout << "Connected to Duco controller " << ip << ":" << port << "\n\n";
  print_pose("TCP pose (base frame)", tcp);
  print_pose("Flange pose (base frame)", flange);

  std::vector<double> tcp_off;
  robot.get_tcp_offset(tcp_off);
  if (tcp_off.size() >= 6) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nActive TCP offset [x,y,z,rx,ry,rz] (m,rad): ";
    for (size_t i = 0; i < tcp_off.size(); ++i) {
      std::cout << tcp_off[i] << (i + 1 < tcp_off.size() ? ", " : "\n");
    }
  }

  robot.close();
  return 0;
}

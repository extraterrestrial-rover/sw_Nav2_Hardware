#include "my_robot_hardware/serial_hardware_interface.hpp"

// Linux serial port headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_hardware
{

// ─────────────────────────────────────────────────────────────────────────────
// on_init – called once when the hardware interface is loaded
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn SerialHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Read parameters from URDF <hardware> section ──────────────────────────
  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";

  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate")) : 115200;

  wheel_radius_ = info_.hardware_parameters.count("wheel_radius")
    ? std::stod(info_.hardware_parameters.at("wheel_radius")) : 0.05;

  // Maximum motor speed in rad/s (at PWM = 255).
  // Tune this to match your actual motors.
  max_motor_speed_rad_s_ = info_.hardware_parameters.count("max_motor_speed_rad_s")
    ? std::stod(info_.hardware_parameters.at("max_motor_speed_rad_s")) : 10.0;

  // Mock hardware mode: skip serial port entirely (for bench testing without ESP32)
  use_mock_hardware_ = info_.hardware_parameters.count("use_mock_hardware")
    ? (info_.hardware_parameters.at("use_mock_hardware") == "true") : false;

  // ── Validate joints ───────────────────────────────────────────────────────
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("SerialHardwareInterface"),
      "Expected exactly 2 joints (left_wheel_joint, right_wheel_joint), got %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialHardwareInterface"),
        "Joint '%s' must have exactly one 'velocity' command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialHardwareInterface"),
        "Joint '%s' must have 'position' and 'velocity' state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ── Allocate state / command buffers ─────────────────────────────────────
  hw_positions_.assign(2, 0.0);
  hw_velocities_.assign(2, 0.0);
  hw_commands_.assign(2, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("SerialHardwareInterface"),
    "Initialized. Port: %s  Baud: %d  WheelR: %.3f  MaxSpeed: %.1f rad/s  MockHW: %s",
    serial_port_.c_str(), baud_rate_, wheel_radius_, max_motor_speed_rad_s_,
    use_mock_hardware_ ? "YES" : "NO");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure – open the serial port
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn SerialHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ── Mock hardware mode: skip serial entirely ─────────────────────────────
  if (use_mock_hardware_) {
    RCLCPP_WARN(
      rclcpp::get_logger("SerialHardwareInterface"),
      "[MOCK] use_mock_hardware=true — serial port disabled. Commands will be simulated.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!open_serial()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SerialHardwareInterface"),
      "Failed to open serial port '%s': %s",
      serial_port_.c_str(), strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SerialHardwareInterface"),
    "Serial port '%s' opened successfully.", serial_port_.c_str());

  // Give ESP32 time to boot (it resets on USB connect)
  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  // Send stop command to make sure robot starts stationary
  send_command(0, 0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn SerialHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_positions_.assign(2, 0.0);
  hw_velocities_.assign(2, 0.0);
  hw_commands_.assign(2, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("SerialHardwareInterface"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate – stop motors safely
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn SerialHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_command(0, 0);
  RCLCPP_INFO(rclcpp::get_logger("SerialHardwareInterface"), "Hardware deactivated (motors stopped).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_cleanup – close serial port
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn SerialHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
SerialHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint 0 = left_wheel_joint
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);

  // Joint 1 = right_wheel_joint
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_positions_[1]);
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);

  return state_interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
SerialHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  command_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);

  return command_interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// read – update state estimates (open-loop: use commanded velocity)
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type SerialHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // No encoders → estimate state from commanded velocity (open-loop integration).
  // These "fake" encoder positions are fed into diff_drive_controller, which 
  // passes them to our EKF filter to fuse with the ZED IMU for robot odometry.
  hw_velocities_[0] = hw_commands_[0];
  hw_velocities_[1] = hw_commands_[1];
  hw_positions_[0] += hw_commands_[0] * period.seconds();
  hw_positions_[1] += hw_commands_[1] * period.seconds();

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write – convert velocity commands to PWM and send over serial
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type SerialHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // In mock mode, commands are accepted but not sent anywhere
  if (use_mock_hardware_) {
    return hardware_interface::return_type::OK;
  }

  // hw_commands_[0] = left  wheel velocity (rad/s)
  // hw_commands_[1] = right wheel velocity (rad/s)

  // Scale rad/s → PWM [-255, 255]
  double scale = 255.0 / max_motor_speed_rad_s_;
  int left_pwm  = static_cast<int>(std::round(hw_commands_[0] * scale));
  int right_pwm = static_cast<int>(std::round(hw_commands_[1] * scale));

  left_pwm  = std::clamp(left_pwm,  -255, 255);
  right_pwm = std::clamp(right_pwm, -255, 255);

  if (left_pwm==-1*right_pwm){
    if (left_pwm>0){
      left_pwm = 200;
      right_pwm = -200;
    }
    else{
      left_pwm = -200;
      right_pwm = 200;
    }
  }

  if (!send_command(left_pwm, right_pwm)) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("SerialHardwareInterface"),
      *rclcpp::Clock::make_shared(), 2000,
      "Serial write failed — check USB connection.");
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

bool SerialHardwareInterface::open_serial()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // ── Baud rate ─────────────────────────────────────────────────────────────
  speed_t baud = B115200;
  switch (baud_rate_) {
    case 9600:   baud = B9600;   break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    default:     baud = B115200; break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // ── 8N1, no flow control ──────────────────────────────────────────────────
  tty.c_cflag  =  (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
  tty.c_cflag &= ~(PARENB | PARODD);              // No parity
  tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                        // No hardware flow ctrl
  tty.c_cflag |=  (CLOCAL | CREAD);               // Enable receiver

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag  = 0;   // Raw output
  tty.c_lflag  = 0;   // Raw input (no echo, no canonical, no signals)

  tty.c_cc[VMIN]  = 0;   // Non-blocking read
  tty.c_cc[VTIME] = 5;   // 0.5 s read timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIFLUSH);
  return true;
}

void SerialHardwareInterface::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("SerialHardwareInterface"), "Serial port closed.");
  }
}

bool SerialHardwareInterface::send_command(int left_pwm, int right_pwm)
{
  if (serial_fd_ < 0) {
    return false;
  }

  // Protocol: "CMD:<left>,<right>\n"
  // left, right: signed integers -255 .. +255
  std::ostringstream oss;
  oss << "CMD:" << left_pwm << "," << right_pwm << "\n";
  std::string cmd = oss.str();

  ssize_t written = ::write(serial_fd_, cmd.c_str(), cmd.size());
  return (written == static_cast<ssize_t>(cmd.size()));
}

}  // namespace my_robot_hardware

// ── Export the plugin ─────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::SerialHardwareInterface,
  hardware_interface::SystemInterface)

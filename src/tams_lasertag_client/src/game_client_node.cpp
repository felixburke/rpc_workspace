#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <sstream>
#include <utility>

#include <irobot_create_msgs/srv/e_stop.hpp>

#include "tams_lasertag_client/srv/submit_hit.hpp"

using namespace std::chrono_literals;
using SubmitHit = tams_lasertag_client::srv::SubmitHit;
using json = nlohmann::json;


namespace
{
size_t write_string_cb(void* contents, size_t size, size_t nmemb, void* userp)
{
  size_t total = size * nmemb;
  std::string* s = static_cast<std::string*>(userp);
  s->append(static_cast<char*>(contents), total);
  return total;
}

class CurlGlobal
{
public:
  CurlGlobal() { curl_global_init(CURL_GLOBAL_DEFAULT); }
  ~CurlGlobal() { curl_global_cleanup(); }
};

// Helper to POST JSON
bool http_post_json(const std::string& url,
                    const json& body,
                    long& http_code_out,
                    std::string& response_out,
                    long timeout_ms = 3000)
{
  CURL* curl = curl_easy_init();
  if (!curl) return false;

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  std::string body_str = body.dump();

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body_str.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body_str.size());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_string_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_out);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  if (res == CURLE_OK) {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  http_code_out = http_code;
  return res == CURLE_OK;
}

// Helper to POST multipart/form-data with one file field + text fields
bool http_post_multipart(const std::string& url,
                         const std::vector<std::pair<std::string, std::string>>& fields,
                         const std::string& file_field_name,
                         const std::string& filename,
                         const std::string& content_type,
                         const std::vector<unsigned char>& file_data,
                         long& http_code_out,
                         std::string& response_out,
                         long timeout_ms = 5000)
{
  CURL* curl = curl_easy_init();
  if (!curl) return false;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_string_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_out);

  curl_mime* mime = curl_mime_init(curl);

  // Add text fields
  for (const auto& kv : fields) {
    curl_mimepart* part = curl_mime_addpart(mime);
    curl_mime_name(part, kv.first.c_str());
    curl_mime_data(part, kv.second.c_str(), CURL_ZERO_TERMINATED);
  }

  // Add file field
  {
    curl_mimepart* part = curl_mime_addpart(mime);
    curl_mime_name(part, file_field_name.c_str());
    curl_mime_filename(part, filename.c_str());
    if (!content_type.empty()) {
      curl_mime_type(part, content_type.c_str());
    }
    // Provide data from memory
    curl_mime_data(part, reinterpret_cast<const char*>(file_data.data()),
                   static_cast<size_t>(file_data.size()));
  }

  curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  if (res == CURLE_OK) {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  }

  curl_mime_free(mime);
  curl_easy_cleanup(curl);

  http_code_out = http_code;
  return res == CURLE_OK;
}

std::string to_camera_matrix_json_string(const sensor_msgs::msg::CameraInfo& ci)
{
  // CameraInfo.K is row-major 3x3
  // [fx, s, cx,
  //  0, fy, cy,
  //  0,  0,  1]
  json mat = json::array({
    json::array({ci.k[0], ci.k[1], ci.k[2]}),
    json::array({ci.k[3], ci.k[4], ci.k[5]}),
    json::array({ci.k[6], ci.k[7], ci.k[8]})
  });
  return mat.dump();
}

} // namespace

class GameClientNode : public rclcpp::Node
{
public:
  GameClientNode()
  : Node("game_client_node")
  {
    // Params
    server_url_ = this->declare_parameter<std::string>("server_url", "http://localhost:5000");

    // Get hostname
    std::string hostname;
    char hostname_buf[256];
    if (gethostname(hostname_buf, sizeof(hostname_buf)) == 0) {
        hostname = hostname_buf;
    } else {
        // Panic
        RCLCPP_FATAL(get_logger(), "Failed to get hostname. Could not initialize GameClientNode.");
        throw std::runtime_error("Failed to get hostname");
    }

    user_ = hostname; // Use hostname as user
    double hb_period = this->declare_parameter<double>("heartbeat_period", 1.0);

    if (!ends_with(server_url_, "/")) {
      server_url_ += "/";
    }

    curl_global_ = std::make_unique<CurlGlobal>();

    // Service
    submit_srv_ = this->create_service<SubmitHit>(
      "submit_hit",
      std::bind(&GameClientNode::on_submit, this, std::placeholders::_1, std::placeholders::_2));

    // Service Clients
    e_stop_srv_ = this->create_client<irobot_create_msgs::srv::EStop>("e_stop");

    // Heartbeat
    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(hb_period),
      std::bind(&GameClientNode::do_heartbeat, this));

    RCLCPP_INFO(get_logger(), "GameClientNode up. server_url=%s, user=%s, hb=%.2fs",
      server_url_.c_str(), user_.c_str(), hb_period);
  }

private:
  static bool ends_with(const std::string& s, const std::string& suffix)
  {
    if (s.size() < suffix.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
  }

  void on_submit(const std::shared_ptr<SubmitHit::Request> req,
                 std::shared_ptr<SubmitHit::Response> resp)
  {
    // Convert ROS Image to JPEG
    std::vector<unsigned char> jpeg_buf;
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(req->image, req->image.encoding);
      cv::Mat mat;

      // Convert to BGR for JPEG if needed
      if (cv_ptr->encoding == "bgr8") {
        mat = cv_ptr->image;
      } else if (cv_ptr->encoding == "rgb8") {
        cv::cvtColor(cv_ptr->image, mat, cv::COLOR_RGB2BGR);
      } else if (cv_ptr->encoding == "mono8") {
        cv::cvtColor(cv_ptr->image, mat, cv::COLOR_GRAY2BGR);
      } else {
        // Fallback: try to convert to BGR
        try {
          mat = cv_bridge::toCvCopy(req->image, "bgr8")->image;
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "Unsupported encoding '%s', trying raw JPEG encode.",
                      req->image.encoding.c_str());
          mat = cv_ptr->image.clone();
        }
      }

      std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 90 };
      if (!cv::imencode(".jpg", mat, jpeg_buf, params)) {
        throw std::runtime_error("cv::imencode(.jpg) failed");
      }
    } catch (const std::exception& e) {
      resp->message = std::string("Image conversion error: ") + e.what();
      return;
    }

    // Build multipart fields
    std::vector<std::pair<std::string, std::string>> fields;
    fields.emplace_back("user", user_);
    fields.emplace_back("camera_matrix", to_camera_matrix_json_string(req->camera_info));

    // HTTP POST /submit
    long code = 0;
    std::string response_body;
    const std::string url = server_url_ + "submit";
    bool ok = http_post_multipart(url, fields,
                                  "image", "frame.jpg", "image/jpeg",
                                  jpeg_buf, code, response_body, 10000);

    if (!ok) {
      resp->message = "HTTP error while posting /submit";
      return;
    }

    if (code / 100 != 2) {
      // Non-2xx
      // Try to extract error message
      std::string msg = "Server returned " + std::to_string(code);
      try {
        auto j = json::parse(response_body);
        if (j.contains("error")) {
          msg += ": " + j["error"].dump();
        } else if (j.contains("errors")) {
          msg += ": " + j["errors"].dump();
        }
      } catch (...) {
        // ignore
      }
      resp->message = msg;
      return;
    }

    // Parse JSON
    try {
      auto j = json::parse(response_body);
      resp->success = true;
      resp->message = "OK";
      resp->hit = j.value("hit", false);
      resp->score = j.value("score", 0.0);
      resp->score_distance = j.value("score_distance", 0.0);
      resp->score_center = j.value("score_center", 0.0);
      resp->score_alignment = j.value("score_alignment", 0.0);
      // server adds target as string in our adaptation
      if (j.contains("target")) {
        if (j["target"].is_string()) {
          resp->target = j["target"].get<std::string>();
        } else {
          // sometimes may be enum/string; fallback to dump
          resp->target = j["target"].dump();
        }
      } else {
        resp->target = "";
      }
      RCLCPP_INFO(get_logger(), "Submit result: hit=%s score=%.2f target=%s",
                  resp->hit ? "true" : "false", resp->score, resp->target.c_str());
    } catch (const std::exception& e) {
      resp->message = std::string("JSON parse error: ") + e.what();
    }
  }

  void do_heartbeat()
  {
    json body = {
      {"user", user_}
    };

    long code = 0;
    std::string resp;
    const std::string url = server_url_ + "heartbeat";
    bool ok = http_post_json(url, body, code, resp, 3000);

    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Heartbeat HTTP error");
      return;
    }
    if (code / 100 != 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Heartbeat returned HTTP %ld: %s", code, resp.c_str());
      return;
    }

    try {
      auto j = json::parse(resp);
      std::string state = j.value("state", "unknown");
      if (state != last_state_) {
        RCLCPP_INFO(get_logger(), "Server state changed: %s -> %s",
                    last_state_.c_str(), state.c_str());
        if (state == "stop") {
          RCLCPP_INFO(get_logger(), "Game stopped, triggering E-Stop");
          // Call E-Stop service
          auto e_stop_req = std::make_shared<irobot_create_msgs::srv::EStop::Request>();
          e_stop_req->e_stop_on = true;
          e_stop_srv_->async_send_request(e_stop_req);
        } else {
          RCLCPP_INFO(get_logger(), "Resetting E-Stop, due to game state change");
          // We can do this safely since the turtlebot is not really dangerous don't do this with bigger robots
          // Reset E-Stop
          auto e_stop_req = std::make_shared<irobot_create_msgs::srv::EStop::Request>();
          e_stop_req->e_stop_on = false;
          e_stop_srv_->async_send_request(e_stop_req);
        }
        last_state_ = state;
      }
      // Optionally, track connected players or scoreboard if needed.
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Heartbeat JSON parse error: %s", e.what());
    }
  }

  std::unique_ptr<CurlGlobal> curl_global_;
  std::string server_url_;
  std::string user_;
  std::string last_state_{"unknown"};

  rclcpp::Service<SubmitHit>::SharedPtr submit_srv_;
  rclcpp::Client<irobot_create_msgs::srv::EStop>::SharedPtr e_stop_srv_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GameClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

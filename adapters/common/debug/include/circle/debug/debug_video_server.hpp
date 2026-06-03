#pragma once

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace circle::debug {

struct DebugHttpRequest {
  std::string method;
  std::string path;
  std::string query;
  std::string body;
};

/**
 * HTTP control plane for debug dashboards and video preview.
 *
 * Provides multipart MJPEG at /api/video/mjpeg (browser-friendly preview) plus
 * H.264/WebRTC signaling hooks; WebRTC media path is wired by downstream nodes.
 */
class DebugVideoServer {
 public:
  using SeriesJsonFn = std::function<std::string(const std::string& query)>;
  using ParamsJsonFn = std::function<std::string()>;
  using PostParamFn =
      std::function<std::pair<int, std::string>(const std::string& body)>;
  using SaveParamsFn = std::function<std::pair<int, std::string>()>;
  using ParamsDownloadFn = std::function<void(int fd, const std::string& query)>;
  using PlotsHtmlFn = std::function<std::string()>;
  using VideoStatusFn = std::function<std::string()>;
  using WebrtcOfferFn =
      std::function<std::string(const std::string& body,
                                const std::string& client_ip)>;

  DebugVideoServer() = default;
  ~DebugVideoServer() { stop(); }

  DebugVideoServer(const DebugVideoServer&) = delete;
  DebugVideoServer& operator=(const DebugVideoServer&) = delete;

  void setSaveParamsHandler(SaveParamsFn handler) {
    save_params_fn_ = std::move(handler);
  }

  void setParamsDownloadHandler(ParamsDownloadFn handler) {
    params_download_fn_ = std::move(handler);
  }

  void setHandlers(PlotsHtmlFn plots_html, SeriesJsonFn series_json,
                   ParamsJsonFn params_json, PostParamFn post_param,
                   VideoStatusFn video_status, bool param_post_enabled) {
    plots_html_fn_ = std::move(plots_html);
    series_json_fn_ = std::move(series_json);
    params_json_fn_ = std::move(params_json);
    post_param_fn_ = std::move(post_param);
    video_status_fn_ = std::move(video_status);
    param_post_enabled_ = param_post_enabled;
  }

  void setWebrtcOfferHandler(WebrtcOfferFn handler) {
    webrtc_offer_fn_ = std::move(handler);
  }

  void setSeriesSessionHz(double hz) {
    const double bounded = std::clamp(hz, 0.2, 20.0);
    const int period_ms =
        std::clamp(static_cast<int>(1000.0 / bounded + 0.5), 50, 5000);
    series_session_period_ms_.store(period_ms);
  }

  bool start(int port, bool bind_localhost = false) {
    if (running_.load()) return false;

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;

    int opt = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr =
        bind_localhost ? htonl(INADDR_LOOPBACK) : htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(listen_fd_, reinterpret_cast<struct sockaddr*>(&addr),
               sizeof(addr)) < 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    if (::listen(listen_fd_, 128) < 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }

    port_ = port;
    bind_localhost_ = bind_localhost;
    running_.store(true);
    accept_thread_ = std::thread(&DebugVideoServer::acceptLoop, this);
    return true;
  }

  void stop() {
    running_.store(false);
    if (listen_fd_ >= 0) {
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();
  }

  bool isRunning() const { return running_.load(); }
  int port() const { return port_; }
  bool bindLocalhost() const { return bind_localhost_; }
  /** EventSource clients on /api/video/session (telemetry only). */
  int activeVideoClients() const { return active_video_clients_.load(); }
  /** EventSource clients on /api/series/session (plots telemetry). */
  int activeSeriesClients() const { return active_series_clients_.load(); }
  bool hasActiveVideoClients() const { return activeVideoClients() > 0; }
  /** Long-lived GET /api/video/mjpeg connections. */
  int activeMjpegClients() const { return mjpeg_stream_clients_.load(); }
  int totalPreviewClients() const {
    return active_video_clients_.load() + mjpeg_stream_clients_.load();
  }
  bool hasPreviewClients() const { return totalPreviewClients() > 0; }

  /**
   * Latest JPEG for MJPEG preview; safe to call from the node's render thread.
   */
  void publishMjpegFrame(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    std::lock_guard<std::mutex> lk(mjpeg_mutex_);
    mjpeg_latest_.assign(data, data + len);
    ++mjpeg_seq_;
  }

 private:
  class ActiveVideoClientGuard {
   public:
    explicit ActiveVideoClientGuard(std::atomic<int>& counter)
        : counter_(counter) {
      counter_.fetch_add(1);
    }
    ~ActiveVideoClientGuard() { counter_.fetch_sub(1); }

   private:
    std::atomic<int>& counter_;
  };

  std::atomic<bool> running_{false};
  int listen_fd_{-1};
  int port_{8080};
  bool bind_localhost_{false};
  std::atomic<int> active_video_clients_{0};
  std::atomic<int> active_series_clients_{0};
  std::atomic<int> mjpeg_stream_clients_{0};
  std::atomic<int> series_session_period_ms_{250};

  std::mutex mjpeg_mutex_;
  std::vector<uint8_t> mjpeg_latest_;
  uint64_t mjpeg_seq_{0};

  std::thread accept_thread_;

  PlotsHtmlFn plots_html_fn_;
  SeriesJsonFn series_json_fn_;
  ParamsJsonFn params_json_fn_;
  PostParamFn post_param_fn_;
  SaveParamsFn save_params_fn_;
  ParamsDownloadFn params_download_fn_;
  VideoStatusFn video_status_fn_;
  WebrtcOfferFn webrtc_offer_fn_;
  std::atomic<bool> param_post_enabled_{false};

  static int parseContentLength(const std::string& headers) {
    constexpr char k[] = "content-length:";
    for (size_t i = 0; i + sizeof(k) - 1 <= headers.size(); ++i) {
      bool match = true;
      for (size_t j = 0; j < sizeof(k) - 1; ++j) {
        if (std::tolower(static_cast<unsigned char>(headers[i + j])) !=
            static_cast<unsigned char>(k[j])) {
          match = false;
          break;
        }
      }
      if (!match) continue;
      size_t p = i + sizeof(k) - 1;
      while (p < headers.size() &&
             (headers[p] == ' ' || headers[p] == '\t')) {
        ++p;
      }
      int len = 0;
      while (p < headers.size() &&
             std::isdigit(static_cast<unsigned char>(headers[p]))) {
        len = len * 10 + (headers[p] - '0');
        ++p;
      }
      return len;
    }
    return 0;
  }

  static bool readHttpRequest(int fd, DebugHttpRequest& out) {
    std::string raw;
    raw.reserve(4096);
    char buf[4096];
    const int kMaxTotal = 262144;
    int content_length = -1;

    while (static_cast<int>(raw.size()) < kMaxTotal) {
      const int n = ::recv(fd, buf, sizeof(buf), 0);
      if (n <= 0) return false;
      raw.append(buf, static_cast<size_t>(n));

      const size_t hdr_end = raw.find("\r\n\r\n");
      if (hdr_end == std::string::npos) continue;

      if (content_length < 0) {
        const std::string headers = raw.substr(0, hdr_end);
        content_length = parseContentLength(headers);
      }
      const size_t body_start = hdr_end + 4;
      if (raw.size() < body_start + static_cast<size_t>(content_length)) {
        continue;
      }
      const std::string request_line_area = raw.substr(0, hdr_end);
      size_t line_end = request_line_area.find("\r\n");
      if (line_end == std::string::npos) return false;
      std::string first = request_line_area.substr(0, line_end);
      size_t sp1 = first.find(' ');
      if (sp1 == std::string::npos) return false;
      size_t sp2 = first.find(' ', sp1 + 1);
      if (sp2 == std::string::npos) sp2 = first.size();
      out.method = first.substr(0, sp1);
      std::string target = first.substr(sp1 + 1, sp2 - sp1 - 1);
      size_t q = target.find('?');
      if (q != std::string::npos) {
        out.path = target.substr(0, q);
        out.query = target.substr(q + 1);
      } else {
        out.path = target;
      }
      out.body = raw.substr(body_start, static_cast<size_t>(content_length));
      return true;
    }
    return false;
  }

  static void configureDebugTcpSocket(int fd) {
    // Debug HTTP/SSE is best-effort telemetry. Mark it as background traffic and
    // avoid large TCP write queues so it does not compete with WebRTC RTP bursts
    // on marginal Wi-Fi links.
    int tos = 0x20;  // DSCP CS1 / scavenger on networks that honor WMM.
    ::setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    int sndbuf = 24 * 1024;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    int nodelay = 0;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

#ifdef TCP_NOTSENT_LOWAT
    int lowat = 4 * 1024;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NOTSENT_LOWAT, &lowat, sizeof(lowat));
#endif
  }

  void acceptLoop() {
    while (running_.load()) {
      struct pollfd pfd{};
      pfd.fd = listen_fd_;
      pfd.events = POLLIN;
      int ret = ::poll(&pfd, 1, 500);
      if (ret <= 0) continue;

      struct sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_fd = ::accept(listen_fd_,
          reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);
      if (client_fd < 0) continue;

      configureDebugTcpSocket(client_fd);
      char client_ip[INET_ADDRSTRLEN] = {0};
      const char* ip_text = ::inet_ntop(AF_INET, &client_addr.sin_addr,
                                        client_ip, sizeof(client_ip));
      std::thread(&DebugVideoServer::handleClient, this, client_fd,
                  std::string(ip_text ? ip_text : "")).detach();
    }
  }

  void handleClient(int fd, std::string client_ip) {
    DebugHttpRequest req;
    if (!readHttpRequest(fd, req)) {
      ::close(fd);
      return;
    }

    if (req.method == "GET") {
      if (req.path == "/" || req.path == "/plots") {
        servePlots(fd);
      } else if (req.path == "/api/series.json") {
        serveSeriesJson(fd, req.query);
      } else if (req.path == "/api/series/session") {
        serveSeriesSession(fd, req.query);
      } else if (req.path == "/api/params.json") {
        serveParamsJson(fd);
      } else if (req.path == "/api/params/save/download") {
        serveParamsSaveDownload(fd, req.query);
      } else if (req.path == "/api/video/status") {
        serveVideoStatus(fd);
      } else if (req.path == "/api/video/session") {
        serveVideoSession(fd);
      } else if (req.path == "/api/video/mjpeg") {
        serveMjpegStream(fd);
        ::close(fd);
        return;
      } else {
        servePlots(fd);
      }
    } else if (req.method == "POST" && req.path == "/api/param") {
      servePostParam(fd, req.body);
    } else if (req.method == "POST" && req.path == "/api/params/save") {
      servePostParamsSave(fd);
    } else if (req.method == "POST" && req.path == "/api/webrtc/offer") {
      serveWebrtcOffer(fd, req.body, client_ip);
    } else {
      const char* resp =
          "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
      sendAll(fd, resp, static_cast<int>(std::strlen(resp)));
    }
    ::close(fd);
  }

  void servePlots(int fd) {
    std::string body =
        plots_html_fn_ ? plots_html_fn_()
                       : std::string("<!DOCTYPE html><html><body>/plots "
                                     "unconfigured</body></html>\n");
    sendTextResponse(fd, "text/html; charset=utf-8", body);
  }

  void serveSeriesJson(int fd, const std::string& query) {
    std::string body = series_json_fn_ ? series_json_fn_(query)
                                       : std::string("{\"error\":\"no data\"}");
    sendTextResponse(fd, "application/json", body);
  }

  void serveSeriesSession(int fd, const std::string& query) {
    ActiveVideoClientGuard guard(active_series_clients_);
    const char* header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n\r\n";
    if (!sendAll(fd, header, static_cast<int>(std::strlen(header)))) return;
    int update_count = 0;
    while (running_.load()) {
      std::string body = series_json_fn_ ? series_json_fn_(query)
                                         : std::string("{\"error\":\"no data\"}");
      for (char& c : body) {
        if (c == '\n' || c == '\r') c = ' ';
      }
      std::string ev = std::string("event: series\ndata: ") + body + "\n\n";
      if (!sendAll(fd, ev.c_str(), static_cast<int>(ev.size()))) return;
      ++update_count;
      std::this_thread::sleep_for(
          std::chrono::milliseconds(series_session_period_ms_.load()));
    }
  }

  void serveParamsJson(int fd) {
    std::string body = params_json_fn_ ? params_json_fn_()
                                       : std::string("{\"params\":[]}");
    sendTextResponse(fd, "application/json", body);
  }

  void serveVideoStatus(int fd) {
    std::string body = video_status_fn_ ? video_status_fn_()
        : std::string("{\"ok\":false,\"error\":\"video status unavailable\"}");
    sendTextResponse(fd, "application/json", body);
  }

  void serveWebrtcOffer(int fd, const std::string& body,
                        const std::string& client_ip) {
    std::string out =
        webrtc_offer_fn_
            ? webrtc_offer_fn_(body, client_ip)
            : std::string(
                  "{\"ok\":false,\"error\":\"webrtc signaling not configured\"}");
    sendTextResponse(fd, "application/json", out);
  }

  void serveVideoSession(int fd) {
    ActiveVideoClientGuard guard(active_video_clients_);
    const char* header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (!sendAll(fd, header, static_cast<int>(std::strlen(header)))) return;
    int update_count = 0;
    while (running_.load()) {
      std::string body = video_status_fn_ ? video_status_fn_()
          : std::string("{\"ok\":false,\"error\":\"video status unavailable\"}");
      std::string ev = "event: status\ndata: " + body + "\n\n";
      if (!sendAll(fd, ev.c_str(), static_cast<int>(ev.size()))) return;
      // Use fast initial status updates so WebRTC can start promptly, then
      // settle to low-rate telemetry; browser WebRTC stats are local.
      ++update_count;
      const int period_ms = update_count <= 12 ? 250 : 1000;
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
  }

  void serveMjpegStream(int fd) {
    ActiveVideoClientGuard guard(mjpeg_stream_clients_);
    uint64_t frames_sent = 0;
    uint64_t bytes_sent = 0;
    std::string end_reason{"server stopping"};
    std::cerr << "debug_video_server/mjpeg: client_start fd=" << fd
              << " active=" << mjpeg_stream_clients_.load() << '\n';
    int sndbuf = 256 * 1024;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    const char* header =
        "HTTP/1.1 200 OK\r\n"
        "Cache-Control: no-cache, no-store\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    if (!sendAllRealtime(fd, header, static_cast<int>(std::strlen(header)), 100)) {
      std::cerr << "debug_video_server/mjpeg: client_end fd=" << fd
                << " reason=header_send_failed frames=0 bytes=0\n";
      return;
    }

    constexpr int kIdlePeriodMs = 10;
    constexpr int kSendTimeoutMs = 1000;
    uint64_t last_sent_seq = 0;
    while (running_.load()) {
      std::vector<uint8_t> chunk;
      uint64_t seq = 0;
      {
        std::lock_guard<std::mutex> lk(mjpeg_mutex_);
        seq = mjpeg_seq_;
        if (seq != last_sent_seq) {
          chunk = mjpeg_latest_;
        }
      }
      if (!chunk.empty()) {
        char head[192];
        const int hlen = snprintf(head, sizeof(head),
                                  "--frame\r\n"
                                  "Content-Type: image/jpeg\r\n"
                                  "Content-Length: %zu\r\n\r\n",
                                  chunk.size());
        if (hlen <= 0 || hlen >= static_cast<int>(sizeof(head))) {
          end_reason = "bad_part_header";
          break;
        }
        if (!sendAllRealtime(fd, head, hlen, kSendTimeoutMs)) {
          end_reason = "part_header_send_failed";
          break;
        }
        if (!sendAllRealtime(fd, reinterpret_cast<const char*>(chunk.data()),
                             static_cast<int>(chunk.size()),
                             kSendTimeoutMs)) {
          end_reason = "jpeg_send_failed";
          break;
        }
        if (!sendAllRealtime(fd, "\r\n", 2, kSendTimeoutMs)) {
          end_reason = "part_tail_send_failed";
          break;
        }
        last_sent_seq = seq;
        ++frames_sent;
        bytes_sent += chunk.size();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kIdlePeriodMs));
    }
    std::cerr << "debug_video_server/mjpeg: client_end fd=" << fd
              << " reason=" << end_reason
              << " frames=" << frames_sent
              << " bytes=" << bytes_sent
              << " active=" << mjpeg_stream_clients_.load() << '\n';
  }

  void serveParamsSaveDownload(int fd, const std::string& query) {
    if (!param_post_enabled_.load() || !params_download_fn_) {
      const char* resp =
          "HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\n"
          "Connection: close\r\n\r\n"
          "{\"ok\":false,\"error\":\"params download unavailable\"}";
      sendAll(fd, resp, static_cast<int>(std::strlen(resp)));
      return;
    }
    params_download_fn_(fd, query);
  }

  void servePostParamsSave(int fd) {
    if (!param_post_enabled_.load() || !save_params_fn_) {
      const char* resp =
          "HTTP/1.1 403 Forbidden\r\nContent-Type: application/json\r\n"
          "Connection: close\r\n\r\n"
          "{\"ok\":false,\"error\":\"params save API disabled\"}";
      sendAll(fd, resp, static_cast<int>(std::strlen(resp)));
      return;
    }
    auto [code, json_body] = save_params_fn_();
    int status = (code >= 200 && code < 600) ? code : 500;
    const char* phrase = (status == 200) ? "OK" : "Internal Server Error";
    std::string hdr = "HTTP/1.1 " + std::to_string(status) + " " + phrase + "\r\n"
        "Content-Type: application/json\r\n"
        "Connection: close\r\n"
        "Content-Length: " + std::to_string(json_body.size()) + "\r\n\r\n";
    sendAll(fd, hdr.c_str(), static_cast<int>(hdr.size()));
    sendAll(fd, json_body.c_str(), static_cast<int>(json_body.size()));
  }

  void servePostParam(int fd, const std::string& body) {
    if (!param_post_enabled_.load() || !post_param_fn_) {
      const char* resp =
          "HTTP/1.1 403 Forbidden\r\nContent-Type: application/json\r\n"
          "Connection: close\r\n\r\n"
          "{\"ok\":false,\"error\":\"param API disabled\"}";
      sendAll(fd, resp, static_cast<int>(std::strlen(resp)));
      return;
    }
    auto [code, json_body] = post_param_fn_(body);
    int status = (code >= 200 && code < 600) ? code : 500;
    const char* phrase = "OK";
    if (status == 400) phrase = "Bad Request";
    else if (status == 403) phrase = "Forbidden";
    else if (status == 404) phrase = "Not Found";
    else if (status >= 500) phrase = "Internal Server Error";
    std::string hdr = "HTTP/1.1 " + std::to_string(status) + " " + phrase + "\r\n"
        "Content-Type: application/json\r\n"
        "Connection: close\r\n"
        "Content-Length: " + std::to_string(json_body.size()) + "\r\n\r\n";
    sendAll(fd, hdr.c_str(), static_cast<int>(hdr.size()));
    sendAll(fd, json_body.c_str(), static_cast<int>(json_body.size()));
  }

  void sendTextResponse(int fd, const char* content_type,
                        const std::string& body) {
    char header[384];
    int hlen = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-store\r\n\r\n",
        content_type, body.size());
    sendAll(fd, header, hlen);
    sendAll(fd, body.c_str(), static_cast<int>(body.size()));
  }

  static bool sendAll(int fd, const char* data, int len) {
    int sent = 0;
    while (sent < len) {
      int r = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
      if (r <= 0) return false;
      sent += r;
    }
    return true;
  }

  static bool sendAllRealtime(int fd, const char* data, int len,
                              int timeout_ms) {
    int sent = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (sent < len) {
      int r = ::send(fd, data + sent, len - sent,
                     MSG_NOSIGNAL | MSG_DONTWAIT);
      if (r > 0) {
        sent += r;
        continue;
      }
      if (r < 0 && errno == EINTR) {
        continue;
      }
      if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return false;
        const auto remain_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now).count();
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLOUT;
        const int wait_ms = static_cast<int>(
            remain_ms > 0 ? std::min<int64_t>(remain_ms, 10) : 0);
        if (::poll(&pfd, 1, wait_ms) <= 0) return false;
        continue;
      }
      return false;
    }
    return true;
  }
};

}  // namespace circle::debug

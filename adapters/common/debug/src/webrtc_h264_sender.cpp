#include "circle/debug/webrtc_h264_sender.hpp"

#include <chrono>
#include <cstring>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#define GST_USE_UNSTABLE_API
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/sdp.h>
#include <gst/video/video-event.h>
#include <gst/webrtc/rtcsessiondescription.h>
#include <gst/webrtc/webrtc.h>

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace circle::debug {

namespace {

std::string JsonEscape(const std::string& s) {
  std::string o;
  o.reserve(s.size() + 16);
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
      o += static_cast<char>(c);
    } else if (c == '\n') {
      o += "\\n";
    } else if (c == '\r') {
    } else {
      o += static_cast<char>(c);
    }
  }
  return o;
}

bool ExtractSdpFromOfferJson(const std::string& body, std::string* sdp_text,
                             std::string* err) {
  size_t k = body.find("\"sdp\"");
  if (k == std::string::npos) {
    if (err) *err = "missing sdp field";
    return false;
  }
  size_t colon = body.find(':', k);
  if (colon == std::string::npos) {
    if (err) *err = "malformed json (sdp)";
    return false;
  }
  size_t i = colon + 1;
  while (i < body.size() && (body[i] == ' ' || body[i] == '\t')) ++i;
  if (i >= body.size() || body[i] != '"') {
    if (err) *err = "sdp value not a string";
    return false;
  }
  ++i;
  std::string raw;
  while (i < body.size()) {
    char c = body[i++];
    if (c == '\\' && i < body.size()) {
      char e = body[i++];
      if (e == 'n')
        raw.push_back('\n');
      else if (e == 'r')
        raw.push_back('\r');
      else if (e == 't')
        raw.push_back('\t');
      else if (e == '"' || e == '\\')
        raw.push_back(e);
      else
        raw.push_back(e);
    } else if (c == '"') {
      break;
    } else {
      raw.push_back(c);
    }
  }
  if (raw.empty()) {
    if (err) *err = "empty sdp";
    return false;
  }
  *sdp_text = std::move(raw);
  return true;
}

std::string MakeErrorJson(const std::string& e) {
  std::ostringstream o;
  o << "{\"ok\":false,\"error\":\"" << JsonEscape(e) << "\"}";
  return o.str();
}

std::string MakeAnswerJson(const std::string& sdp) {
  std::ostringstream o;
  o << "{\"ok\":true,\"type\":\"answer\",\"sdp\":\"" << JsonEscape(sdp) << "\"}";
  return o.str();
}

bool WebRtcH264DebugLogEnabled() {
  static const bool enabled = []() {
    const char* v = std::getenv("CIRCLE_PILOT_WEBRTC_H264_LOG");
    if (!v || !*v) return false;
    const std::string value(v);
    return value != "0" && value != "false" && value != "FALSE" &&
           value != "off" && value != "OFF" && value != "no" &&
           value != "NO";
  }();
  return enabled;
}

int ExtractPayloadType(const std::string& line, const char* prefix) {
  const std::string p(prefix);
  if (line.rfind(p, 0) != 0) return -1;
  const size_t start = p.size();
  size_t end = start;
  while (end < line.size() && line[end] >= '0' && line[end] <= '9') ++end;
  if (end == start) return -1;
  return std::atoi(line.substr(start, end - start).c_str());
}

bool OfferHasH264Payload(const std::string& sdp, int payload_type) {
  std::istringstream in(sdp);
  std::string line;
  const std::string prefix = "a=rtpmap:" + std::to_string(payload_type) + " ";
  while (std::getline(in, line)) {
    if (line.rfind(prefix, 0) == 0 &&
        line.find("H264/90000") != std::string::npos) {
      return true;
    }
  }
  return false;
}

int PreferredH264PayloadTypeFromOffer(const std::string& sdp) {
  std::istringstream first_pass(sdp);
  std::string line;
  while (std::getline(first_pass, line)) {
    const int pt = ExtractPayloadType(line, "a=fmtp:");
    if (pt >= 0 && line.find("packetization-mode=1") != std::string::npos &&
        OfferHasH264Payload(sdp, pt)) {
      return pt;
    }
  }

  std::istringstream second_pass(sdp);
  while (std::getline(second_pass, line)) {
    const int pt = ExtractPayloadType(line, "a=rtpmap:");
    if (pt >= 0 && line.find("H264/90000") != std::string::npos) {
      return pt;
    }
  }
  return -1;
}

std::vector<std::string> SplitWhitespace(const std::string& s) {
  std::istringstream in(s);
  std::vector<std::string> out;
  std::string tok;
  while (in >> tok) {
    out.push_back(tok);
  }
  return out;
}

bool LooksLikeMdnsOrHostname(const std::string& addr) {
  if (addr.find(".local") != std::string::npos) return true;
  for (char c : addr) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
      return true;
    }
  }
  return false;
}

std::string JoinWhitespace(const std::vector<std::string>& parts) {
  std::string out;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i) out += ' ';
    out += parts[i];
  }
  return out;
}

std::string Ipv4SubnetPrefix24(const std::string& ip) {
  const size_t last_dot = ip.rfind('.');
  if (last_dot == std::string::npos || last_dot == 0) {
    return {};
  }
  return ip.substr(0, last_dot);
}

bool SameIpv4Subnet24(const std::string& a, const std::string& b) {
  const std::string pa = Ipv4SubnetPrefix24(a);
  const std::string pb = Ipv4SubnetPrefix24(b);
  return !pa.empty() && pa == pb;
}

bool IsLoopbackClientIp(const std::string& ip) {
  return ip == "127.0.0.1" || ip == "::1" || ip.rfind("127.", 0) == 0;
}

bool IsExcludedLanHost(const std::string& addr, bool prefer_wifi_lan) {
  // Orange Pi dual-NIC: USB/tether 192.168.12.x breaks WebRTC when browser uses
  // 192.168.100.x WiFi. Drop 12.x whenever a 100.x host candidate exists.
  if (prefer_wifi_lan && addr.rfind("192.168.12.", 0) == 0) {
    return true;
  }
  return false;
}

/** Drop host ICE candidates on other NICs (multi-homed boards). */
std::string FilterAnswerSdpToClientSubnet(const std::string& sdp,
                                          const std::string& client_ip) {
  bool has_wifi_lan_host = false;
  {
    std::istringstream scan(sdp);
    std::string line;
    while (std::getline(scan, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.rfind("a=candidate:", 0) != 0) {
        continue;
      }
      const std::vector<std::string> parts =
          SplitWhitespace(line.substr(12));
      if (parts.size() >= 8 && parts.back() == "host" &&
          parts[4].rfind("192.168.100.", 0) == 0) {
        has_wifi_lan_host = true;
        break;
      }
    }
  }

  const bool peer_loopback = IsLoopbackClientIp(client_ip);
  const bool filter_by_peer_subnet =
      !client_ip.empty() && !peer_loopback &&
      !Ipv4SubnetPrefix24(client_ip).empty();

  std::string host_ip;
  std::ostringstream out;
  std::istringstream in(sdp);
  std::string line;
  int dropped = 0;
  int kept_host = 0;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.rfind("a=candidate:", 0) == 0) {
      const std::vector<std::string> parts =
          SplitWhitespace(line.substr(12));
      if (parts.size() >= 8) {
        const std::string& addr = parts[4];
        const std::string& typ = parts.back();
        if (typ == "host") {
          bool drop = false;
          if (IsExcludedLanHost(addr, has_wifi_lan_host)) {
            drop = true;
          } else if (filter_by_peer_subnet &&
                     !SameIpv4Subnet24(addr, client_ip)) {
            drop = true;
          } else if (peer_loopback && addr.rfind("192.168.12.", 0) == 0) {
            drop = true;
          }
          if (drop) {
            ++dropped;
            continue;
          }
          ++kept_host;
          if (host_ip.empty()) {
            host_ip = addr;
          }
        }
      }
      out << line << "\r\n";
      continue;
    }
    if (line.rfind("c=IN IP4 ", 0) == 0) {
      if (!host_ip.empty()) {
        out << "c=IN IP4 " << host_ip << "\r\n";
      } else {
        out << line << "\r\n";
      }
      continue;
    }
    out << line << "\r\n";
  }

  std::cerr << "[webrtc_h264] ICE filter peer=" << client_ip
            << " loopback_peer=" << (peer_loopback ? "yes" : "no")
            << " dropped=" << dropped << " kept_host=" << kept_host
            << " keep_ip=" << (host_ip.empty() ? "?" : host_ip) << '\n';
  if (kept_host == 0) {
    std::cerr << "[webrtc_h264] WARNING: no host ICE candidates left after "
                 "filter; browser WebRTC will fail\n";
    return sdp;
  }
  return out.str();
}

int AddIceCandidatesFromOfferSdp(GstElement* webrtc, const std::string& sdp,
                                 const std::string& client_ip) {
  if (!webrtc) return 0;
  std::istringstream in(sdp);
  std::string line;
  int current_mline = -1;
  int count = 0;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.rfind("m=", 0) == 0) {
      ++current_mline;
      continue;
    }
    if (line.rfind("a=candidate:", 0) != 0) {
      continue;
    }
    const guint mline = static_cast<guint>(std::max(current_mline, 0));
    // webrtcbin's add-ice-candidate signal expects the SDP attribute value
    // without the leading "a=".
    std::string candidate = line.substr(2);
    std::vector<std::string> parts = SplitWhitespace(candidate);
    if (parts.size() >= 8) {
      const std::string original_addr = parts[4];
      if (!client_ip.empty() && LooksLikeMdnsOrHostname(original_addr)) {
        parts[4] = client_ip;
        candidate = JoinWhitespace(parts);
        if (WebRtcH264DebugLogEnabled()) {
          std::cerr << "[webrtc_h264] remote ICE candidate mDNS/host "
                    << original_addr << " rewritten to HTTP peer " << client_ip
                    << " port=" << parts[5] << std::endl;
        }
      } else {
        if (WebRtcH264DebugLogEnabled()) {
          std::cerr << "[webrtc_h264] remote ICE candidate addr="
                    << original_addr << " port=" << parts[5]
                    << " typ=" << parts[7]
                    << " http_peer="
                    << (client_ip.empty() ? "(unknown)" : client_ip)
                    << std::endl;
        }
      }
    }
    g_signal_emit_by_name(webrtc, "add-ice-candidate", mline,
                          candidate.c_str());
    ++count;
  }
  if (WebRtcH264DebugLogEnabled()) {
    std::cerr << "[webrtc_h264] added remote SDP ICE candidates=" << count
              << std::endl;
  }
  return count;
}

bool GstFactoryAvailable(const char* factory_name) {
  GstElementFactory* factory = gst_element_factory_find(factory_name);
  if (!factory) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

void SetIntIfProperty(GstElement* element, const char* name, gint value) {
  GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(element), name);
  if (!pspec) return;
  const GType type = G_PARAM_SPEC_VALUE_TYPE(pspec);
  if (type == G_TYPE_INT) {
    g_object_set(element, name, value, NULL);
  } else if (type == G_TYPE_UINT) {
    g_object_set(element, name, static_cast<guint>(std::max(0, value)), NULL);
  } else if (G_TYPE_IS_ENUM(type)) {
    g_object_set(element, name, value, NULL);
  }
}

void SetUintIfProperty(GstElement* element, const char* name, guint value) {
  GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(element), name);
  if (!pspec) return;
  const GType type = G_PARAM_SPEC_VALUE_TYPE(pspec);
  if (type == G_TYPE_UINT) {
    g_object_set(element, name, value, NULL);
  } else if (type == G_TYPE_INT) {
    g_object_set(element, name, static_cast<gint>(std::min<guint>(value, G_MAXINT)),
                 NULL);
  }
}

void SetBoolIfProperty(GstElement* element, const char* name, gboolean value) {
  GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(element), name);
  if (!pspec) return;
  if (G_PARAM_SPEC_VALUE_TYPE(pspec) == G_TYPE_BOOLEAN) {
    g_object_set(element, name, value, NULL);
  }
}

void SetUint64IfProperty(GstElement* element, const char* name, guint64 value) {
  GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(element), name);
  if (!pspec) return;
  const GType type = G_PARAM_SPEC_VALUE_TYPE(pspec);
  if (type == G_TYPE_UINT64 || type == G_TYPE_ULONG || type == G_TYPE_LONG) {
    g_object_set(element, name, value, NULL);
  } else if (type == G_TYPE_UINT || type == G_TYPE_INT) {
    g_object_set(element, name, static_cast<guint>(std::min<guint64>(value, G_MAXUINT)),
                 NULL);
  }
}

bool BgrToNv12(const cv::Mat& bgr, std::vector<uint8_t>* nv12,
               int* out_w, int* out_h) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || !nv12 || !out_w || !out_h) {
    return false;
  }
  const int w = bgr.cols & ~1;
  const int h = bgr.rows & ~1;
  if (w <= 0 || h <= 0) return false;
  cv::Mat even = (w == bgr.cols && h == bgr.rows) ? bgr : bgr(cv::Rect(0, 0, w, h));
  cv::Mat i420;
  cv::cvtColor(even, i420, cv::COLOR_BGR2YUV_I420);

  const size_t y_size = static_cast<size_t>(w) * h;
  const size_t uv_plane_size = y_size / 4;
  nv12->resize(y_size + 2 * uv_plane_size);
  std::memcpy(nv12->data(), i420.data, y_size);
  const uint8_t* u = i420.data + y_size;
  const uint8_t* v = u + uv_plane_size;
  uint8_t* uv = nv12->data() + y_size;
  for (size_t i = 0; i < uv_plane_size; ++i) {
    uv[2 * i] = u[i];
    uv[2 * i + 1] = v[i];
  }
  *out_w = w;
  *out_h = h;
  return true;
}

void UpdateMaxAtomic(std::atomic<uint64_t>& target, uint64_t value) {
  uint64_t cur = target.load(std::memory_order_relaxed);
  while (value > cur &&
         !target.compare_exchange_weak(cur, value, std::memory_order_relaxed)) {
  }
}

void ConfigureEncoder(GstElement* enc, const std::string& factory_name,
                      int bitrate_kbps, int fps_int) {
  if (factory_name.find("vaapi") != std::string::npos) {
    g_object_set(enc, "bitrate", bitrate_kbps, NULL);
    SetIntIfProperty(enc, "rate-control", 2 /* cbr */);
    SetIntIfProperty(enc, "max-bframes", 0);
    SetIntIfProperty(enc, "b-frames", 0);
  } else if (factory_name.find("nvh264") != std::string::npos) {
    g_object_set(enc, "bitrate", bitrate_kbps * 1000, NULL);
    SetBoolIfProperty(enc, "zerolatency", TRUE);
    SetIntIfProperty(enc, "bframes", 0);
    SetIntIfProperty(enc, "b-frames", 0);
    SetIntIfProperty(enc, "rc-mode", 2 /* cbr where supported */);
  } else if (factory_name.find("mpp") != std::string::npos) {
    // GstMppH264Profile: baseline=66, main=77, high=100 (default). Value 0 is invalid.
    // bps/bps-max/bps-min on mpph264enc are guint, so set them with explicit
    // unsigned values to avoid GValue assertion warnings from GLib.
    const guint bps = static_cast<guint>(bitrate_kbps) * 1000u;
    g_object_set(enc, "bps", bps, NULL);
    // Cap peak bitrate so an IDR frame cannot burst more than ~1.3x the
    // average. Without this, mpph264enc lets IDR NALUs balloon 4-6x the
    // P-frame size, which on Wi-Fi shows up as bursty RTP loss every GOP
    // (~20% packets_lost cumulative even at 600 kbps average).
    if (g_object_class_find_property(G_OBJECT_GET_CLASS(enc), "bps-max")) {
      g_object_set(enc, "bps-max", static_cast<guint>(bps * 13u / 10u), NULL);
    }
    if (g_object_class_find_property(G_OBJECT_GET_CLASS(enc), "bps-min")) {
      g_object_set(enc, "bps-min", static_cast<guint>(bps * 7u / 10u), NULL);
    }
    SetIntIfProperty(enc, "profile", 66 /* baseline for browser WebRTC */);
    SetIntIfProperty(enc, "level", 31);
    // Constrain IDR QP so I-frames don't blow past bps-max either.
    SetIntIfProperty(enc, "qp-max-i", 42);
    SetIntIfProperty(enc, "qp-min-i", 26);
    // WebRTC receivers may join after the encoder's first frame; repeat SPS/PPS
    // on every IDR so the browser can initialize its H.264 decoder immediately.
    SetIntIfProperty(enc, "header-mode", 1 /* each-idr */);
    SetIntIfProperty(enc, "rc-mode", 1 /* cbr */);
    SetIntIfProperty(enc, "bframes", 0);
    SetIntIfProperty(enc, "b-frames", 0);
    SetBoolIfProperty(enc, "low-latency", TRUE);
    SetBoolIfProperty(enc, "maxperf-enable", TRUE);
    // Frequent IDR: H.264 P-frames cannot recover after UDP loss on WiFi.
    if (fps_int > 0) {
      const gint frames = std::clamp(fps_int / 3, 2, 6);
      SetIntIfProperty(enc, "gop", frames);
      SetIntIfProperty(enc, "idr-interval", 1);
      SetIntIfProperty(enc, "key-int-max", frames);
    } else {
      SetIntIfProperty(enc, "gop", 4);
    }
  } else {
    g_object_set(enc, "bitrate", bitrate_kbps, NULL);
    SetIntIfProperty(enc, "bframes", 0);
    SetIntIfProperty(enc, "b-frames", 0);
  }
}

void SendForceKeyUnit(GstElement* enc) {
  if (!enc) return;
  // Upstream-direction events must be delivered on a SRC pad; sending one on a
  // sink pad fails with "custom-upstream event in wrong direction" and the
  // encoder never produces a fresh IDR.
  GstPad* srcpad = gst_element_get_static_pad(enc, "src");
  if (!srcpad) return;
  GstEvent* ev = gst_video_event_new_upstream_force_key_unit(
      GST_CLOCK_TIME_NONE, /*all_headers=*/TRUE, /*count=*/0);
  gst_pad_send_event(srcpad, ev);
  gst_object_unref(srcpad);
}

void OnIceConnectionStateChanged(GstElement* webrtc, GParamSpec* /*pspec*/,
                                 gpointer user_data) {
  auto* enc = static_cast<GstElement*>(user_data);
  GstWebRTCICEConnectionState state = GST_WEBRTC_ICE_CONNECTION_STATE_NEW;
  g_object_get(webrtc, "ice-connection-state", &state, NULL);
  if (WebRtcH264DebugLogEnabled()) {
    std::cerr << "[webrtc_h264] ice-connection-state="
              << static_cast<int>(state) << std::endl;
  }
  if (state == GST_WEBRTC_ICE_CONNECTION_STATE_CONNECTED ||
      state == GST_WEBRTC_ICE_CONNECTION_STATE_COMPLETED) {
    SendForceKeyUnit(enc);
  }
}

void OnWebRtcEnumPropertyChanged(GObject* obj, GParamSpec* pspec,
                                 gpointer /*user_data*/) {
  if (!WebRtcH264DebugLogEnabled()) {
    return;
  }
  const char* name = g_param_spec_get_name(pspec);
  GValue value = G_VALUE_INIT;
  g_value_init(&value, G_PARAM_SPEC_VALUE_TYPE(pspec));
  g_object_get_property(obj, name, &value);
  if (G_VALUE_HOLDS_ENUM(&value)) {
    std::cerr << "[webrtc_h264] " << name << "=" << g_value_get_enum(&value)
              << std::endl;
  } else if (G_VALUE_HOLDS_UINT(&value)) {
    std::cerr << "[webrtc_h264] " << name << "=" << g_value_get_uint(&value)
              << std::endl;
  } else if (G_VALUE_HOLDS_INT(&value)) {
    std::cerr << "[webrtc_h264] " << name << "=" << g_value_get_int(&value)
              << std::endl;
  }
  g_value_unset(&value);
}

bool WaitIceGatheringComplete(GstElement* webrtc, GMainContext* ctx, int timeout_ms) {
  GstWebRTCICEGatheringState state = GST_WEBRTC_ICE_GATHERING_STATE_NEW;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    g_object_get(webrtc, "ice-gathering-state", &state, NULL);
    if (state == GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE) {
      return true;
    }
    if (ctx) {
      g_main_context_iteration(ctx, FALSE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  g_object_get(webrtc, "ice-gathering-state", &state, NULL);
  return state == GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE;
}

const char* GstStateName(GstState state) {
  switch (state) {
    case GST_STATE_VOID_PENDING:
      return "void-pending";
    case GST_STATE_NULL:
      return "null";
    case GST_STATE_READY:
      return "ready";
    case GST_STATE_PAUSED:
      return "paused";
    case GST_STATE_PLAYING:
      return "playing";
  }
  return "unknown";
}

gboolean OnBusMessage(GstBus* /*bus*/, GstMessage* msg, gpointer /*user_data*/) {
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError* err = nullptr;
      gchar* dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      std::cerr << "[webrtc_h264] ERROR from "
                << GST_OBJECT_NAME(msg->src) << ": "
                << (err ? err->message : "(unknown)")
                << (dbg ? " debug=" : "") << (dbg ? dbg : "") << std::endl;
      if (err) g_error_free(err);
      g_free(dbg);
      break;
    }
    case GST_MESSAGE_WARNING: {
      if (!WebRtcH264DebugLogEnabled()) break;
      GError* err = nullptr;
      gchar* dbg = nullptr;
      gst_message_parse_warning(msg, &err, &dbg);
      std::cerr << "[webrtc_h264] WARNING from "
                << GST_OBJECT_NAME(msg->src) << ": "
                << (err ? err->message : "(unknown)")
                << (dbg ? " debug=" : "") << (dbg ? dbg : "") << std::endl;
      if (err) g_error_free(err);
      g_free(dbg);
      break;
    }
    case GST_MESSAGE_STATE_CHANGED: {
      if (!WebRtcH264DebugLogEnabled()) break;
      if (GST_IS_ELEMENT(msg->src)) {
        const char* name = GST_OBJECT_NAME(msg->src);
        if (name && (std::string(name) == "webrtc-send" ||
                     std::string(name) == "src" ||
                     std::string(name) == "enc" ||
                     std::string(name) == "pay" ||
                     std::string(name) == "webrtc")) {
          GstState old_state;
          GstState new_state;
          GstState pending;
          gst_message_parse_state_changed(msg, &old_state, &new_state, &pending);
          std::cerr << "[webrtc_h264] state " << name << ": "
                    << GstStateName(old_state) << " -> "
                    << GstStateName(new_state) << " pending="
                    << GstStateName(pending) << std::endl;
        }
      }
      break;
    }
    default:
      break;
  }
  return G_SOURCE_CONTINUE;
}

GstPadProbeReturn OnRtpBufferProbe(GstPad* /*pad*/, GstPadProbeInfo* info,
                                   gpointer user_data) {
  if (!(GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }
  auto* count = static_cast<std::atomic<uint64_t>*>(user_data);
  const uint64_t n = count->fetch_add(1, std::memory_order_relaxed) + 1;
  if (WebRtcH264DebugLogEnabled() && (n <= 5 || n % 300 == 0)) {
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    std::cerr << "[webrtc_h264] rtp #" << n << " bytes="
              << (buffer ? gst_buffer_get_size(buffer) : 0) << std::endl;
  }
  return GST_PAD_PROBE_OK;
}

GstPadProbeReturn OnH264BufferProbe(GstPad* /*pad*/, GstPadProbeInfo* info,
                                    gpointer user_data) {
  if (!(GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }
  auto* count = static_cast<std::atomic<uint64_t>*>(user_data);
  const uint64_t n = count->fetch_add(1, std::memory_order_relaxed) + 1;
  if (WebRtcH264DebugLogEnabled() && (n <= 5 || n % 300 == 0)) {
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    std::cerr << "[webrtc_h264] h264 #" << n << " bytes="
              << (buffer ? gst_buffer_get_size(buffer) : 0) << std::endl;
  }
  return GST_PAD_PROBE_OK;
}

GstWebRTCSessionDescription* TakeAnswerFromPromise(GstPromise* promise,
                                                   std::string* err) {
  const GstStructure* reply = gst_promise_get_reply(promise);
  if (!reply) {
    if (err) *err = "create_answer: empty reply";
    gst_promise_unref(promise);
    return nullptr;
  }
  GstWebRTCSessionDescription* answer = nullptr;
  if (!gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION,
                          &answer, nullptr)) {
    if (err) *err = "create_answer: missing answer field";
    gst_promise_unref(promise);
    return nullptr;
  }
  gst_promise_unref(promise);
  return answer;
}

}  // namespace

struct WebRtcH264Sender::Impl {
  GMainLoop* loop = nullptr;
  GMainContext* ctx = nullptr;
  std::thread loop_thread;
  GstElement* pipeline = nullptr;
  GstElement* webrtc = nullptr;
  GstElement* appsrc_el = nullptr;
  GstPad* webrtc_sink_pad = nullptr;
  int frame_w = 0;
  int frame_h = 0;
  std::mutex pipeline_mutex;
  uint64_t pts_frame = 0;
  uint64_t push_count = 0;
  std::atomic<uint64_t> h264_count{0};
  std::atomic<uint64_t> rtp_count{0};
  GstFlowReturn last_push_ret = GST_FLOW_OK;
  std::mutex latest_mutex;
  cv::Mat latest_bgr;
  uint64_t latest_seq = 0;
  uint64_t consumed_seq = 0;
  bool push_scheduled = false;
  std::chrono::steady_clock::time_point latest_accept_time{};
  std::atomic<uint64_t> frames_accepted{0};
  std::atomic<uint64_t> frames_dropped{0};
  std::atomic<uint64_t> frames_pushed{0};
  std::atomic<uint64_t> bytes_pushed{0};
  std::atomic<uint64_t> push_errors{0};
  std::atomic<int> last_flow_return{static_cast<int>(GST_FLOW_OK)};
  std::atomic<uint64_t> last_queue_wait_us{0};
  std::atomic<uint64_t> last_convert_us{0};
  std::atomic<uint64_t> last_push_us{0};
  std::atomic<uint64_t> max_queue_wait_us{0};
  std::atomic<uint64_t> max_convert_us{0};
  std::atomic<uint64_t> max_push_us{0};
};

std::string WebRtcH264Sender::GstElementFromEncoderBackend(
    const std::string& backend_name) {
  static std::once_flag gst_once;
  std::call_once(gst_once, []() { gst_init(nullptr, nullptr); });

  if (backend_name == "h264_gstreamer_mpp") return "mpph264enc";
  if (backend_name == "h264_gstreamer_vaapi") return "vaapih264enc";
  if (backend_name == "h264_gstreamer_nvenc") return "nvh264enc";
  if (backend_name == "auto" || backend_name == "h264_platform_auto" ||
      backend_name.empty()) {
    if (GstFactoryAvailable("mpph264enc")) return "mpph264enc";
    if (GstFactoryAvailable("vaapih264enc")) return "vaapih264enc";
    if (GstFactoryAvailable("nvh264enc")) return "nvh264enc";
  }
  return {};
}

WebRtcH264Sender::WebRtcH264Sender(Config config)
    : impl_(std::make_unique<Impl>()), config_(std::move(config)) {
  media_fps_.store(config_.fps > 0.1 ? config_.fps : 10.0);
}

WebRtcH264Sender::~WebRtcH264Sender() { stop(); }

void WebRtcH264Sender::setFps(double fps) {
  double v = fps > 0.1 ? fps : 10.0;
  if (v < 1.0) {
    v = 1.0;
  }
  if (v > 60.0) {
    v = 60.0;
  }
  media_fps_.store(v);
  std::lock_guard<std::mutex> lk(err_mutex_);
  config_.fps = v;
}

void WebRtcH264Sender::setError(std::string msg) {
  std::lock_guard<std::mutex> lk(err_mutex_);
  last_error_ = std::move(msg);
}

std::string WebRtcH264Sender::lastError() const {
  std::lock_guard<std::mutex> lk(err_mutex_);
  return last_error_;
}

bool WebRtcH264Sender::start() {
  if (running_.load() && impl_->loop) return true;
  if (config_.encoder_element.empty()) {
    setError("encoder_element empty");
    return false;
  }
  static std::once_flag gst_once;
  std::call_once(gst_once, []() { gst_init(nullptr, nullptr); });
  if (!GstFactoryAvailable("webrtcbin")) {
    setError("GStreamer webrtcbin plugin missing (install gstreamer1.0-plugins-bad)");
    return false;
  }
  if (!GstFactoryAvailable("nicesrc") || !GstFactoryAvailable("nicesink")) {
    setError("GStreamer libnice plugin missing (install gstreamer1.0-nice)");
    return false;
  }
  if (!GstFactoryAvailable(config_.encoder_element.c_str())) {
    setError("GStreamer encoder plugin missing: " + config_.encoder_element);
    return false;
  }

  impl_->ctx = g_main_context_new();
  g_main_context_push_thread_default(impl_->ctx);
  impl_->loop = g_main_loop_new(impl_->ctx, FALSE);
  g_main_context_pop_thread_default(impl_->ctx);

  impl_->loop_thread = std::thread([this]() {
    g_main_context_push_thread_default(impl_->ctx);
    g_main_loop_run(impl_->loop);
    g_main_context_pop_thread_default(impl_->ctx);
  });

  running_.store(true);
  return true;
}

void WebRtcH264Sender::stop() {
  const bool was_running = running_.exchange(false);
  if (!was_running && impl_->loop == nullptr) return;

  {
    std::lock_guard<std::mutex> lk(impl_->pipeline_mutex);
    impl_->appsrc_el = nullptr;
  }

  if (impl_->ctx && impl_->loop) {
    g_main_context_invoke(
        impl_->ctx,
        +[](gpointer user_data) -> gboolean {
          auto* self = static_cast<WebRtcH264Sender*>(user_data);
          {
            std::lock_guard<std::mutex> lk(self->impl_->pipeline_mutex);
            if (self->impl_->pipeline) {
              gst_element_set_state(self->impl_->pipeline, GST_STATE_NULL);
              if (self->impl_->webrtc_sink_pad && self->impl_->webrtc) {
                gst_element_release_request_pad(self->impl_->webrtc,
                                                self->impl_->webrtc_sink_pad);
                gst_object_unref(self->impl_->webrtc_sink_pad);
                self->impl_->webrtc_sink_pad = nullptr;
              }
              gst_object_unref(self->impl_->pipeline);
              self->impl_->pipeline = nullptr;
              self->impl_->webrtc = nullptr;
              self->impl_->appsrc_el = nullptr;
            }
          }
          g_main_loop_quit(self->impl_->loop);
          return G_SOURCE_REMOVE;
        },
        this);
  }

  if (impl_->loop_thread.joinable()) {
    impl_->loop_thread.join();
  }
  if (impl_->loop) {
    g_main_loop_unref(impl_->loop);
    impl_->loop = nullptr;
  }
  if (impl_->ctx) {
    g_main_context_unref(impl_->ctx);
    impl_->ctx = nullptr;
  }
}

void WebRtcH264Sender::pushBgrFrame(const cv::Mat& bgr) {
  if (!running_.load() || bgr.empty() || bgr.type() != CV_8UC3) return;

  {
    std::lock_guard<std::mutex> lk(err_mutex_);
    impl_->frame_w = bgr.cols & ~1;
    impl_->frame_h = bgr.rows & ~1;
  }

  cv::Mat packed = bgr.isContinuous() ? bgr : bgr.clone();
  {
    std::lock_guard<std::mutex> lk(impl_->latest_mutex);
    if (!impl_->latest_bgr.empty() && impl_->latest_seq != impl_->consumed_seq) {
      impl_->frames_dropped.fetch_add(1, std::memory_order_relaxed);
    }
    impl_->latest_bgr = packed.clone();
    ++impl_->latest_seq;
    impl_->latest_accept_time = std::chrono::steady_clock::now();
    impl_->frames_accepted.fetch_add(1, std::memory_order_relaxed);
  }
  scheduleLatestFramePush();
}

void WebRtcH264Sender::scheduleLatestFramePush() {
  if (!running_.load() || !impl_->ctx) return;
  {
    std::lock_guard<std::mutex> lk(impl_->latest_mutex);
    if (impl_->push_scheduled) return;
    impl_->push_scheduled = true;
  }
  g_main_context_invoke(
      impl_->ctx,
      +[](gpointer user_data) -> gboolean {
        static_cast<WebRtcH264Sender*>(user_data)->pushLatestFrameOnGstThread();
        return G_SOURCE_REMOVE;
      },
      this);
}

void WebRtcH264Sender::pushLatestFrameOnGstThread() {
  cv::Mat bgr;
  uint64_t seq = 0;
  std::chrono::steady_clock::time_point accepted_at{};
  {
    std::lock_guard<std::mutex> lk(impl_->latest_mutex);
    bgr = std::move(impl_->latest_bgr);
    seq = impl_->latest_seq;
    accepted_at = impl_->latest_accept_time;
    impl_->consumed_seq = seq;
    impl_->push_scheduled = false;
  }
  if (bgr.empty()) return;
  if (accepted_at.time_since_epoch().count() != 0) {
    const auto queue_wait_us =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - accepted_at).count());
    impl_->last_queue_wait_us.store(queue_wait_us, std::memory_order_relaxed);
    UpdateMaxAtomic(impl_->max_queue_wait_us, queue_wait_us);
  }

  GstElement* as = nullptr;
  uint64_t frame_index = 0;
  {
    std::lock_guard<std::mutex> lk(impl_->pipeline_mutex);
    as = impl_->appsrc_el;
    if (!as || !impl_->pipeline) {
      impl_->frames_dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    frame_index = impl_->pts_frame++;
    gst_object_ref(as);
  }

  std::vector<uint8_t> nv12;
  int nv12_w = 0;
  int nv12_h = 0;
  const auto convert_start = std::chrono::steady_clock::now();
  if (!BgrToNv12(bgr, &nv12, &nv12_w, &nv12_h)) {
    gst_object_unref(as);
    impl_->push_errors.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const auto convert_us =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - convert_start).count());
  impl_->last_convert_us.store(convert_us, std::memory_order_relaxed);
  UpdateMaxAtomic(impl_->max_convert_us, convert_us);
  const gsize size = static_cast<gsize>(nv12.size());
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    gst_buffer_unref(buffer);
    gst_object_unref(as);
    impl_->push_errors.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  memcpy(map.data, nv12.data(), size);
  gst_buffer_unmap(buffer, &map);

  const double fps_live = media_fps_.load();
  const int fps_i =
      static_cast<int>(fps_live > 0.1 ? fps_live : 10.0);
  const GstClockTime duration =
      gst_util_uint64_scale_int(GST_SECOND, 1, std::max(1, fps_i));
  GST_BUFFER_PTS(buffer) = frame_index * duration;
  GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DURATION(buffer) = duration;
  GST_BUFFER_OFFSET(buffer) = frame_index;

  GstFlowReturn ret = GST_FLOW_ERROR;
  const auto push_start = std::chrono::steady_clock::now();
  g_signal_emit_by_name(as, "push-buffer", buffer, &ret);
  const auto push_us =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - push_start).count());
  impl_->last_push_us.store(push_us, std::memory_order_relaxed);
  UpdateMaxAtomic(impl_->max_push_us, push_us);
  gst_object_unref(as);
  gst_buffer_unref(buffer);

  impl_->last_flow_return.store(static_cast<int>(ret), std::memory_order_relaxed);
  if (ret == GST_FLOW_OK) {
    impl_->frames_pushed.fetch_add(1, std::memory_order_relaxed);
    impl_->bytes_pushed.fetch_add(static_cast<uint64_t>(size),
                                  std::memory_order_relaxed);
  } else {
    impl_->push_errors.fetch_add(1, std::memory_order_relaxed);
  }

  uint64_t n = 0;
  GstFlowReturn last_ret = GST_FLOW_OK;
  {
    std::lock_guard<std::mutex> lk(impl_->pipeline_mutex);
    n = ++impl_->push_count;
    last_ret = impl_->last_push_ret;
    impl_->last_push_ret = ret;
  }
  if (ret != GST_FLOW_OK ||
      (WebRtcH264DebugLogEnabled() &&
       (n <= 5 || n % 300 == 0 || ret != last_ret))) {
    std::cerr << "[webrtc_h264] push #" << n << " size="
              << nv12_w << "x" << nv12_h << " bytes=" << size
              << " ret=" << gst_flow_get_name(ret)
              << " queue_wait_ms="
              << (impl_->last_queue_wait_us.load(std::memory_order_relaxed) /
                  1000.0)
              << " convert_ms=" << (convert_us / 1000.0)
              << " push_ms=" << (push_us / 1000.0) << std::endl;
  }

  bool needs_reschedule = false;
  {
    std::lock_guard<std::mutex> lk(impl_->latest_mutex);
    needs_reschedule = impl_->latest_seq != seq && !impl_->push_scheduled;
  }
  if (needs_reschedule) {
    scheduleLatestFramePush();
  }
}

bool WebRtcH264Sender::hasMediaPipeline() const {
  if (!impl_) return false;
  std::lock_guard<std::mutex> lk(impl_->pipeline_mutex);
  return impl_->pipeline != nullptr && impl_->appsrc_el != nullptr;
}

bool WebRtcH264Sender::hasFrameSize() const {
  if (!impl_) return false;
  std::lock_guard<std::mutex> lk(err_mutex_);
  return impl_->frame_w > 0 && impl_->frame_h > 0;
}

WebRtcH264Sender::Stats WebRtcH264Sender::stats() const {
  Stats s;
  if (!impl_) return s;
  s.frames_accepted = impl_->frames_accepted.load(std::memory_order_relaxed);
  s.frames_dropped = impl_->frames_dropped.load(std::memory_order_relaxed);
  s.frames_pushed = impl_->frames_pushed.load(std::memory_order_relaxed);
  s.bytes_pushed = impl_->bytes_pushed.load(std::memory_order_relaxed);
  s.push_errors = impl_->push_errors.load(std::memory_order_relaxed);
  s.h264_buffers = impl_->h264_count.load(std::memory_order_relaxed);
  s.rtp_packets = impl_->rtp_count.load(std::memory_order_relaxed);
  s.last_flow_return = impl_->last_flow_return.load(std::memory_order_relaxed);
  s.last_queue_wait_ms =
      impl_->last_queue_wait_us.load(std::memory_order_relaxed) / 1000.0;
  s.last_convert_ms =
      impl_->last_convert_us.load(std::memory_order_relaxed) / 1000.0;
  s.last_push_ms = impl_->last_push_us.load(std::memory_order_relaxed) / 1000.0;
  s.max_queue_wait_ms =
      impl_->max_queue_wait_us.load(std::memory_order_relaxed) / 1000.0;
  s.max_convert_ms =
      impl_->max_convert_us.load(std::memory_order_relaxed) / 1000.0;
  s.max_push_ms = impl_->max_push_us.load(std::memory_order_relaxed) / 1000.0;
  {
    std::lock_guard<std::mutex> lk(err_mutex_);
    s.frame_width = impl_->frame_w;
    s.frame_height = impl_->frame_h;
  }
  return s;
}

std::string WebRtcH264Sender::handleOfferOnGstThread(
    const std::string& offer_json, const std::string& client_ip) {
  Impl* im = impl_.get();
  std::string sdp_in;
  std::string parse_err;
  if (!ExtractSdpFromOfferJson(offer_json, &sdp_in, &parse_err)) {
    setError(parse_err);
    return MakeErrorJson(parse_err);
  }
  const int h264_payload_type = PreferredH264PayloadTypeFromOffer(sdp_in);
  if (h264_payload_type < 0) {
    const std::string e = "browser offer did not include H.264";
    setError(e);
    return MakeErrorJson(e);
  }

  std::cerr << "[webrtc_h264] browser_offer peer=" << client_ip
            << " encoder=" << config_.encoder_element << '\n';

  {
    std::lock_guard<std::mutex> lk(im->pipeline_mutex);
    if (im->pipeline) {
      gst_element_set_state(im->pipeline, GST_STATE_NULL);
      if (im->webrtc_sink_pad && im->webrtc) {
        gst_element_release_request_pad(im->webrtc, im->webrtc_sink_pad);
        gst_object_unref(im->webrtc_sink_pad);
        im->webrtc_sink_pad = nullptr;
      }
      gst_object_unref(im->pipeline);
      im->pipeline = nullptr;
      im->webrtc = nullptr;
      im->appsrc_el = nullptr;
      im->pts_frame = 0;
      im->push_count = 0;
      im->h264_count.store(0, std::memory_order_relaxed);
      im->rtp_count.store(0, std::memory_order_relaxed);
      im->last_push_ret = GST_FLOW_OK;
    }
  }

  int w = 0;
  int h = 0;
  const auto frame_size_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  while (std::chrono::steady_clock::now() < frame_size_deadline) {
    {
      std::lock_guard<std::mutex> lk(err_mutex_);
      w = im->frame_w;
      h = im->frame_h;
    }
    if (w > 0 && h > 0) break;
    if (im->ctx) {
      g_main_context_iteration(im->ctx, FALSE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (w <= 0) w = config_.default_width;
  if (h <= 0) h = config_.default_height;
  if (w < 32) w = 32;
  if (h < 32) h = 32;

  GstElement* pipeline = gst_pipeline_new("webrtc-send");
  GstElement* webrtc = gst_element_factory_make("webrtcbin", "webrtc");
  GstElement* appsrc = gst_element_factory_make("appsrc", "src");
  GstElement* queue = gst_element_factory_make("queue", "q");
  GstElement* cf = gst_element_factory_make("capsfilter", "cf");
  GstElement* enc =
      gst_element_factory_make(config_.encoder_element.c_str(), "enc");
  GstElement* parse = gst_element_factory_make("h264parse", "parse");
  GstElement* pay = gst_element_factory_make("rtph264pay", "pay");
  GstElement* rtpcaps = gst_element_factory_make("capsfilter", "rtpcaps");
  GstElement* rtpqueue = gst_element_factory_make("queue", "rtpq");

  if (!pipeline || !webrtc || !appsrc || !queue || !cf || !enc || !parse ||
      !pay || !rtpcaps || !rtpqueue) {
    std::string e = "failed to create GStreamer elements (install plugins?)";
    setError(e);
    if (pipeline) gst_object_unref(pipeline);
    return MakeErrorJson(e);
  }

  const double fps_live = media_fps_.load();
  const int fps_i =
      static_cast<int>(fps_live > 0.1 ? fps_live : 10.0);
  {
    std::lock_guard<std::mutex> lk(im->pipeline_mutex);
    im->pts_frame = 0;
    im->push_count = 0;
    im->h264_count.store(0, std::memory_order_relaxed);
    im->rtp_count.store(0, std::memory_order_relaxed);
    im->last_push_ret = GST_FLOW_OK;
  }
  if (WebRtcH264DebugLogEnabled()) {
    std::cerr << "[webrtc_h264] building pipeline encoder="
              << config_.encoder_element << " size=" << w << "x" << h
              << " fps=" << fps_i << " bitrate_kbps=" << config_.bitrate_kbps
              << " pt=" << h264_payload_type
              << " stun="
              << (config_.stun_server.empty() ? "(none)" : config_.stun_server)
              << std::endl;
  }

  GstCaps* caps_nv12 =
      gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "NV12",
                          "width", G_TYPE_INT, w, "height", G_TYPE_INT, h,
                          "framerate", GST_TYPE_FRACTION, fps_i, 1, NULL);
  g_object_set(cf, "caps", caps_nv12, NULL);
  gst_caps_unref(caps_nv12);

  g_object_set(appsrc, "is-live", TRUE, "format", GST_FORMAT_TIME,
               "do-timestamp", TRUE, NULL);
  SetBoolIfProperty(appsrc, "block", FALSE);
  SetUint64IfProperty(appsrc, "max-buffers", 1);
  SetUint64IfProperty(appsrc, "max-bytes", 0);
  SetUint64IfProperty(appsrc, "max-time", static_cast<guint64>(50 * GST_MSECOND));
  SetIntIfProperty(appsrc, "leaky-type", 2 /* downstream: drop oldest */);

  // Realtime monitor mode: never preserve backlog. Every queue is allowed to
  // discard old data so the browser gets the newest decodable frame as soon as
  // the network / decoder can accept it.
  SetUintIfProperty(queue, "max-size-buffers", 1);
  SetUintIfProperty(queue, "max-size-bytes", 0);
  g_object_set(queue, "max-size-time", static_cast<guint64>(50 * GST_MSECOND),
               "leaky", 2 /* downstream */, NULL);
  SetUintIfProperty(rtpqueue, "max-size-buffers", 32);
  SetUintIfProperty(rtpqueue, "max-size-bytes", 0);
  g_object_set(rtpqueue, "max-size-time", static_cast<guint64>(50 * GST_MSECOND),
               "leaky", 2 /* downstream */, NULL);

  GstCaps* caps_appsrc = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "NV12", "width", G_TYPE_INT, w,
      "height", G_TYPE_INT, h, "framerate", GST_TYPE_FRACTION, fps_i, 1, NULL);
  gst_app_src_set_caps(GST_APP_SRC(appsrc), caps_appsrc);
  gst_caps_unref(caps_appsrc);

  ConfigureEncoder(enc, config_.encoder_element, config_.bitrate_kbps, fps_i);
  if (g_object_class_find_property(G_OBJECT_GET_CLASS(parse), "config-interval")) {
    g_object_set(parse, "config-interval", -1, NULL);
  }
  SetUintIfProperty(pay, "pt", static_cast<guint>(h264_payload_type));
  g_object_set(pay, "config-interval", -1, NULL);
  // RTP MTU: smaller payloads spread one frame across more packets, and with
  // ~10% Wi-Fi packet loss each extra packet roughly doubles the chance of a
  // broken frame. Path MTU on Wi-Fi/Ethernet is 1500; pick 1200 to stay below
  // any UDP-layer fragmentation while halving packets-per-frame vs. 600.
  g_object_set(pay, "mtu", static_cast<guint>(1200), NULL);
  guint pay_mtu = 0;
  g_object_get(pay, "mtu", &pay_mtu, NULL);
  std::cerr << "[webrtc_h264] rtph264pay mtu=" << pay_mtu << std::endl;

  GstCaps* caps_rtp = gst_caps_new_simple(
      "application/x-rtp", "media", G_TYPE_STRING, "video", "encoding-name",
      G_TYPE_STRING, "H264", "payload", G_TYPE_INT, h264_payload_type,
      "clock-rate", G_TYPE_INT, 90000, NULL);
  g_object_set(rtpcaps, "caps", caps_rtp, NULL);
  gst_caps_unref(caps_rtp);

  if (!config_.stun_server.empty()) {
    g_object_set(webrtc, "stun-server", config_.stun_server.c_str(), NULL);
  }
  SetIntIfProperty(webrtc, "latency", 0);
  g_object_set(webrtc, "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, NULL);

  g_signal_connect(webrtc, "notify::ice-connection-state",
                   G_CALLBACK(OnIceConnectionStateChanged), enc);
  g_signal_connect(webrtc, "notify::ice-gathering-state",
                   G_CALLBACK(OnWebRtcEnumPropertyChanged), nullptr);
  g_signal_connect(webrtc, "notify::signaling-state",
                   G_CALLBACK(OnWebRtcEnumPropertyChanged), nullptr);
  if (g_object_class_find_property(G_OBJECT_GET_CLASS(webrtc),
                                   "connection-state")) {
    g_signal_connect(webrtc, "notify::connection-state",
                     G_CALLBACK(OnWebRtcEnumPropertyChanged), nullptr);
  }

  GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch_full(bus, G_PRIORITY_DEFAULT, OnBusMessage, nullptr, nullptr);
  gst_object_unref(bus);

  gst_bin_add_many(GST_BIN(pipeline), webrtc, appsrc, queue, cf, enc, parse,
                   pay, rtpcaps, rtpqueue, NULL);
  if (!gst_element_link_many(appsrc, queue, cf, enc, parse, pay, rtpcaps,
                             rtpqueue, NULL)) {
    gst_object_unref(pipeline);
    setError("failed to link encode chain");
    return MakeErrorJson("failed to link encode chain");
  }

  GstPad* enc_srcpad = gst_element_get_static_pad(enc, "src");
  if (enc_srcpad) {
    gst_pad_add_probe(enc_srcpad, GST_PAD_PROBE_TYPE_BUFFER, OnH264BufferProbe,
                      &im->h264_count, nullptr);
    gst_object_unref(enc_srcpad);
  }

  GstPad* rtp_probe_pad = gst_element_get_static_pad(rtpcaps, "src");
  if (rtp_probe_pad) {
    gst_pad_add_probe(rtp_probe_pad, GST_PAD_PROBE_TYPE_BUFFER,
                      OnRtpBufferProbe, &im->rtp_count, nullptr);
    gst_object_unref(rtp_probe_pad);
  }
  GstPad* srcpad = gst_element_get_static_pad(rtpqueue, "src");
  GstPad* sinkpad = gst_element_request_pad_simple(webrtc, "sink_%u");
  if (!sinkpad) {
    if (srcpad) gst_object_unref(srcpad);
    gst_object_unref(pipeline);
    std::string e =
        "failed to request webrtcbin RTP sink pad (check gstreamer1.0-nice / "
        "libnice runtime plugin)";
    setError(e);
    return MakeErrorJson(e);
  }
  if (!srcpad) {
    gst_object_unref(sinkpad);
    gst_object_unref(pipeline);
    std::string e = "failed to get RTP capsfilter src pad";
    setError(e);
    return MakeErrorJson(e);
  }
  GstPadLinkReturn link_ret =
      gst_pad_link(srcpad, sinkpad);
  if (link_ret != GST_PAD_LINK_OK) {
    gst_object_unref(sinkpad);
    gst_object_unref(srcpad);
    gst_object_unref(pipeline);
    std::string e = "failed to link RTP to webrtcbin: ";
    e += gst_pad_link_get_name(link_ret);
    setError(e);
    return MakeErrorJson(e);
  }
  gst_object_unref(srcpad);

  {
    std::lock_guard<std::mutex> lk(im->pipeline_mutex);
    im->pipeline = pipeline;
    im->webrtc = webrtc;
    // Do not expose appsrc to the ROS image thread until the pipeline is
    // PLAYING; otherwise the first SPS/PPS/IDR can be queued before WebRTC is
    // ready and never reach the browser.
    im->appsrc_el = nullptr;
    im->webrtc_sink_pad = sinkpad;
  }

  if (gst_element_set_state(pipeline, GST_STATE_READY) ==
      GST_STATE_CHANGE_FAILURE) {
    {
      std::lock_guard<std::mutex> lk(im->pipeline_mutex);
      im->pipeline = nullptr;
      im->webrtc = nullptr;
      im->appsrc_el = nullptr;
      im->webrtc_sink_pad = nullptr;
    }
    gst_object_unref(pipeline);
    setError("READY failed");
    return MakeErrorJson("READY failed");
  }

  GstSDPMessage* sdp_msg = nullptr;
  if (gst_sdp_message_new_from_text(sdp_in.c_str(), &sdp_msg) != GST_SDP_OK) {
    setError("invalid SDP offer text");
    return MakeErrorJson("invalid SDP offer text");
  }
  GstWebRTCSessionDescription* offer_gst =
      gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_msg);

  GstPromise* pr = gst_promise_new();
  g_signal_emit_by_name(webrtc, "set-remote-description", offer_gst, pr);
  gst_promise_wait(pr);
  gst_promise_unref(pr);
  gst_webrtc_session_description_free(offer_gst);
  AddIceCandidatesFromOfferSdp(webrtc, sdp_in, client_ip);

  GstPromise* p_ans = gst_promise_new();
  g_signal_emit_by_name(webrtc, "create-answer", nullptr, p_ans);
  gst_promise_wait(p_ans);

  std::string cerr;
  GstWebRTCSessionDescription* answer = TakeAnswerFromPromise(p_ans, &cerr);
  if (!answer) {
    setError(cerr);
    return MakeErrorJson(cerr.empty() ? "create_answer failed" : cerr);
  }

  GstPromise* p_loc = gst_promise_new();
  g_signal_emit_by_name(webrtc, "set-local-description", answer, p_loc);
  gst_promise_wait(p_loc);
  gst_promise_unref(p_loc);

  // Don't fail on gathering timeout: libnice's UPnP-IGD discovery alone can
  // hold the state in GATHERING for many seconds and is unrelated to whether
  // we already have host + srflx candidates good enough for a media path.
  // We wait briefly for the common case (host + srflx ~ <500ms) then return
  // the answer with whatever candidates were inserted into local-description.
  const int ice_wait_ms = config_.stun_server.empty() ? 800 : 1500;
  WaitIceGatheringComplete(webrtc, im->ctx, ice_wait_ms);

  GstWebRTCSessionDescription* local = nullptr;
  g_object_get(webrtc, "local-description", &local, NULL);
  if (!local) {
    setError("no local description after set_local");
    return MakeErrorJson("no local description after set_local");
  }
  GstSDPMessage* answered_sdp = local->sdp;
  gchar* text = gst_sdp_message_as_text(answered_sdp);
  std::string sdp_out(text ? text : "");
  g_free(text);
  gst_webrtc_session_description_free(local);
  sdp_out = FilterAnswerSdpToClientSubnet(sdp_out, client_ip);

  if (gst_element_set_state(pipeline, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    {
      std::lock_guard<std::mutex> lk(im->pipeline_mutex);
      im->pipeline = nullptr;
      im->webrtc = nullptr;
      im->appsrc_el = nullptr;
      im->webrtc_sink_pad = nullptr;
    }
    gst_object_unref(pipeline);
    setError("PLAYING failed");
    return MakeErrorJson("PLAYING failed");
  }

  {
    std::lock_guard<std::mutex> lk(im->pipeline_mutex);
    im->appsrc_el = appsrc;
  }
  if (WebRtcH264DebugLogEnabled()) {
    std::cerr << "[webrtc_h264] appsrc enabled after PLAYING" << std::endl;
  }

  // Belt-and-braces: request several keyframes after PLAYING. Some webrtcbin /
  // mpp combinations connect RTP a little after the first encoder IDR; repeated
  // requests avoid relying on a single timing window.
  for (guint delay_ms : {200U, 800U, 1600U, 3000U}) {
    GSource* timeout_src = g_timeout_source_new(delay_ms);
    auto* enc_ref = GST_ELEMENT(gst_object_ref(enc));
    g_source_set_callback(
        timeout_src,
        +[](gpointer data) -> gboolean {
          SendForceKeyUnit(static_cast<GstElement*>(data));
          return G_SOURCE_REMOVE;
        },
        enc_ref,
        +[](gpointer data) { gst_object_unref(data); });
    g_source_attach(timeout_src, im->ctx);
    g_source_unref(timeout_src);
  }

  setError({});
  return MakeAnswerJson(sdp_out);
}

std::string WebRtcH264Sender::handleBrowserOffer(
    const std::string& offer_json, const std::string& client_ip) {
  if (!running_.load() || !impl_->ctx) {
    return R"({"ok":false,"error":"sender not started"})";
  }

  using Task = std::packaged_task<std::string()>;
  auto task = std::make_shared<Task>([this, offer_json, client_ip]() {
    try {
      return handleOfferOnGstThread(offer_json, client_ip);
    } catch (...) {
      return std::string(R"({"ok":false,"error":"exception in webrtc handler"})");
    }
  });
  std::future<std::string> fut = task->get_future();
  g_main_context_invoke(
      impl_->ctx,
      +[](gpointer data) -> gboolean {
        auto* p = static_cast<std::shared_ptr<Task>*>(data);
        (**p)();
        delete p;
        return G_SOURCE_REMOVE;
      },
      new std::shared_ptr<Task>(task));

  return fut.get();
}

}  // namespace circle::debug

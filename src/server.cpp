#include <cstdint>
#include <map>
#include <vector>

#include <boost/asio.hpp>

#include "spdlog/spdlog.h"

#include "spdlog/fmt/bin_to_hex.h"

#include "protocol.hpp"
#include "server.hpp"

using boost::asio::ip::udp;

Server::Server(boost::asio::io_context *_io_context, Robot *_robot, std::string ip,
               unsigned short port)
    : io_context(_io_context)
    , robot(_robot)
    , socket_(*io_context, ip.size() ? udp::endpoint(ba::ip::make_address(ip), port)
                                     : udp::endpoint(udp::v4(), port))
    , callbacks() {}

std::vector<uint8_t> Server::answer_request(const uint8_t *buffer, size_t length) {
  uint8_t set, id, attri, sender, receiver;
  uint16_t seq_id;
  const uint8_t *payload;
  if (!decode_request(buffer, length, &set, &id, &seq_id, &attri, &sender, &receiver, &payload)) {
    spdlog::warn("Failed to decode request");
    return {};
  }
  unsigned key = key_from(set, id);
  if (!callbacks.count(key)) {
    spdlog::warn("Unknown request with set 0x{:x} and id 0x{:x}", set, id);
    return {};
  }
  return callbacks.at(key)(sender, receiver, seq_id, attri, payload);
}

void Server::has_received_bytes(const uint8_t *raw_request, size_t length) {
  spdlog::debug("Received {} bytes: {:n}", length,
                spdlog::to_hex(std::vector<uint8_t>(raw_request, raw_request + length)));
  const std::vector<uint8_t> raw_response = answer_request(raw_request, length);
  if (!raw_response.size()) {
    spdlog::debug("Empty response");
  } else {
    spdlog::debug("Send back {} bytes: {:n}", raw_response.size(), spdlog::to_hex(raw_response));
  }
  send(raw_response);
}

void Server::do_receive() {
  socket_.async_receive_from(boost::asio::buffer(data_, kMaxLength), sender_endpoint_,
                             [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                               if (!ec && bytes_recvd > 0) {
                                 has_received_bytes(data_, bytes_recvd);
                               }
                               do_receive();
                             });
}

void Server::send(const std::vector<uint8_t> &data) {
  if (data.empty())
    return;
  socket_.async_send_to(boost::asio::buffer(data.data(), data.size()), sender_endpoint_,
                        [](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/) {});
}

void Server::start() { do_receive(); }

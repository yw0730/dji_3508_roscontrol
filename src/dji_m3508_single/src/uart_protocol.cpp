#include "dji_m3508_single/uart_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace dji_m3508_single
{

namespace
{
constexpr uint16_t kCrcInit = 0xFFFF;
constexpr uint16_t kCrcPoly = 0xA001;

void append_le16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_le32(std::vector<uint8_t> & out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

uint16_t read_le16(const uint8_t * data)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8));
}

uint32_t read_le32(const uint8_t * data)
{
  return static_cast<uint32_t>(
    static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24));
}

}  // namespace

uint16_t crc16_ibm(const uint8_t * data, std::size_t size)
{
  uint16_t crc = kCrcInit;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<uint16_t>((crc >> 1) ^ kCrcPoly);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

std::vector<uint8_t> encode_frame(
  uint8_t type,
  uint16_t seq,
  uint32_t timestamp_ms,
  const std::vector<uint8_t> & payload,
  uint8_t version)
{
  std::vector<uint8_t> frame;
  frame.reserve(2 + 10 + payload.size() + 2);

  frame.push_back(kSofByte0);
  frame.push_back(kSofByte1);
  frame.push_back(version);
  frame.push_back(type);
  append_le16(frame, seq);
  append_le32(frame, timestamp_ms);
  append_le16(frame, static_cast<uint16_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  // CRC covers VER..PAYLOAD and excludes SOF and CRC itself.
  const uint16_t crc = crc16_ibm(frame.data() + 2, frame.size() - 2);
  append_le16(frame, crc);

  return frame;
}

void FrameDecoder::reset()
{
  state_ = State::SEEK_SOF_0;
  reset_frame_accumulator();
}

void FrameDecoder::reset_frame_accumulator()
{
  header_buf_.clear();
  payload_buf_.clear();
  crc_buf_.clear();
  expected_payload_len_ = 0;
  current_ = DecodedFrame{};
}

std::vector<DecodedFrame> FrameDecoder::push_bytes(const uint8_t * data, std::size_t size)
{
  std::vector<DecodedFrame> out;

  for (std::size_t i = 0; i < size; ++i) {
    const uint8_t byte = data[i];

    switch (state_) {
      case State::SEEK_SOF_0:
        if (byte == kSofByte0) {
          state_ = State::SEEK_SOF_1;
        }
        break;

      case State::SEEK_SOF_1:
        if (byte == kSofByte1) {
          state_ = State::READ_HEADER;
          reset_frame_accumulator();
        } else if (byte != kSofByte0) {
          state_ = State::SEEK_SOF_0;
        }
        break;

      case State::READ_HEADER:
        header_buf_.push_back(byte);
        if (header_buf_.size() == kHeaderLen) {
          current_.version = header_buf_[0];
          current_.type = header_buf_[1];
          current_.seq = read_le16(&header_buf_[2]);
          current_.timestamp_ms = read_le32(&header_buf_[4]);
          expected_payload_len_ = read_le16(&header_buf_[8]);

          // Basic sanity check: reset parser on unreasonable payload length.
          if (expected_payload_len_ > 256) {
            reset();
          } else if (expected_payload_len_ == 0) {
            state_ = State::READ_CRC;
          } else {
            state_ = State::READ_PAYLOAD;
          }
        }
        break;

      case State::READ_PAYLOAD:
        payload_buf_.push_back(byte);
        if (payload_buf_.size() == expected_payload_len_) {
          state_ = State::READ_CRC;
        }
        break;

      case State::READ_CRC:
        crc_buf_.push_back(byte);
        if (crc_buf_.size() == 2) {
          const uint16_t crc_rx = read_le16(crc_buf_.data());

          std::vector<uint8_t> crc_data;
          crc_data.reserve(header_buf_.size() + payload_buf_.size());
          crc_data.insert(crc_data.end(), header_buf_.begin(), header_buf_.end());
          crc_data.insert(crc_data.end(), payload_buf_.begin(), payload_buf_.end());

          const uint16_t crc_calc = crc16_ibm(crc_data.data(), crc_data.size());
          if (crc_calc == crc_rx) {
            current_.payload = payload_buf_;
            out.push_back(current_);
          }

          state_ = State::SEEK_SOF_0;
          reset_frame_accumulator();
        }
        break;
    }
  }

  return out;
}

}  // namespace dji_m3508_single

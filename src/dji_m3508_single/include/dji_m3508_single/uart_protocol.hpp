#ifndef DJI_M3508_SINGLE__UART_PROTOCOL_HPP_
#define DJI_M3508_SINGLE__UART_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dji_m3508_single
{

constexpr uint8_t kUartVersion = 0x01;
constexpr uint8_t kSofByte0 = 0xAA;
constexpr uint8_t kSofByte1 = 0x55;

enum MessageType : uint8_t
{
  kMsgHeartbeat = 0x01,
  kMsgSetVelocity = 0x10,
  kMsgSetMode = 0x11,
  kMsgMotorState = 0x20,
  kMsgDiagState = 0x21,
  kMsgAck = 0x7F,
};

enum ModeType : uint8_t
{
  kModeDisable = 0,
  kModeVelocity = 1,
};

enum AckResult : uint8_t
{
  kAckOk = 0,
  kAckError = 1,
};

struct DecodedFrame
{
  uint8_t version {kUartVersion};
  uint8_t type {0};
  uint16_t seq {0};
  uint32_t timestamp_ms {0};
  std::vector<uint8_t> payload;
};

uint16_t crc16_ibm(const uint8_t * data, std::size_t size);

std::vector<uint8_t> encode_frame(
  uint8_t type,
  uint16_t seq,
  uint32_t timestamp_ms,
  const std::vector<uint8_t> & payload,
  uint8_t version = kUartVersion);

class FrameDecoder
{
public:
  FrameDecoder() = default;

  void reset();
  std::vector<DecodedFrame> push_bytes(const uint8_t * data, std::size_t size);

private:
  enum class State
  {
    SEEK_SOF_0,
    SEEK_SOF_1,
    READ_HEADER,
    READ_PAYLOAD,
    READ_CRC,
  };

  void reset_frame_accumulator();

  State state_ {State::SEEK_SOF_0};

  static constexpr std::size_t kHeaderLen = 10;  // ver(1)+type(1)+seq(2)+ts(4)+len(2)
  std::vector<uint8_t> header_buf_;
  std::vector<uint8_t> payload_buf_;
  std::vector<uint8_t> crc_buf_;

  uint16_t expected_payload_len_ {0};
  DecodedFrame current_;
};

}  // namespace dji_m3508_single

#endif  // DJI_M3508_SINGLE__UART_PROTOCOL_HPP_

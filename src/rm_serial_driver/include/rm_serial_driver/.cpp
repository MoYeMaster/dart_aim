#include "VisionUsb.hpp"

#include "Bsp_usb.hpp"
#include "Crc.hpp"
#include <cstring>

VisionUsb &VisionUsb::instance()
{
    static VisionUsb instance;
    return instance;
}

void VisionUsb::init()
{
    Usb::instance().registerCallback(
        [this](uint8_t *data, uint16_t length) {
            onReceive(data, length);
        });
}

void VisionUsb::update()
{
    uint8_t data = 0;

    while (popByte(data)) {
        parseByte(data);
    }

    // 临时测试：收到有效数据帧后，将数据原样回传给视觉上位机。
    // 测试协议逻辑放在 Vision 模块内部，AppManager 只负责调度。
    runTestEcho();
    }

void VisionUsb::runTestEcho()
{
    if (!frameReceived_) {
        return;
    }

    uint8_t receivedFrame[FRAME_BUFFER_SIZE] = {};
    uint16_t receivedLength = 0;

    // copyLastFrame() 会消费当前帧，避免同一帧被重复回传。
    if (!copyLastFrame(
            receivedFrame,
            sizeof(receivedFrame),
            receivedLength)) {
        return;
    }

    if (receivedLength != VISION::FRAME_SIZE) {
        return;
    }

    // 帧布局：帧头、flags、16 字节数据段、CRC16。
    // send() 会重新生成帧头并计算 CRC。
    send(receivedFrame[1], &receivedFrame[2]);
}

bool VisionUsb::send(uint8_t flags, const uint8_t *data)
{
    if (data == nullptr) {
        return false;
    }

    txBuffer_[0] = VISION::FRAME_HEAD;
    txBuffer_[1] = flags;
    std::memcpy(&txBuffer_[2], data, VISION::DATA_SIZE);

    const uint16_t crc =
        Get_CRC16_Check_Sum(txBuffer_, VISION::FRAME_SIZE - 2U, 0xffffU);
    txBuffer_[18] = static_cast<uint8_t>(crc & 0xffU);
    txBuffer_[19] = static_cast<uint8_t>(crc >> 8U);

    return Usb::instance().send(txBuffer_, VISION::FRAME_SIZE) == 0;
}

bool VisionUsb::copyLastFrame(
    uint8_t *buffer,
    uint16_t bufferSize,
    uint16_t &length)
{
    if (!frameReceived_ || buffer == nullptr || bufferSize < lastFrameLength_) {
        return false;
    }

    std::memcpy(buffer, lastFrame_, lastFrameLength_);
    length = lastFrameLength_;
    frameReceived_ = false;
    return true;
}

void VisionUsb::onReceive(uint8_t *data, uint16_t length)
{
    if (data == nullptr) {
        return;
    }

    for (uint16_t index = 0; index < length; ++index) {
        const uint16_t nextWriteIndex =
            static_cast<uint16_t>((rxWriteIndex_ + 1U) % RX_BUFFER_SIZE);

        if (nextWriteIndex == rxReadIndex_) {
            break;
        }

        rxBuffer_[rxWriteIndex_] = data[index];
        rxWriteIndex_ = nextWriteIndex;
    }
}

bool VisionUsb::popByte(uint8_t &data)
{
    if (rxReadIndex_ == rxWriteIndex_) {
        return false;
    }

    data = rxBuffer_[rxReadIndex_];
    rxReadIndex_ =
        static_cast<uint16_t>((rxReadIndex_ + 1U) % RX_BUFFER_SIZE);
    return true;
}

void VisionUsb::parseByte(uint8_t data)
{
    if (frameIndex_ == 0 && data != VISION::FRAME_HEAD) {
        return;
    }

    frameBuffer_[frameIndex_++] = data;

    if (frameIndex_ == VISION::FRAME_SIZE) {
        handleFrame();
        resetParser();
    }
}

void VisionUsb::resetParser()
{
    frameIndex_ = 0;
}

void VisionUsb::handleFrame()
{
    if (!Verify_CRC16_Check_Sum(frameBuffer_, frameIndex_)) {
        return;
    }

    std::memcpy(lastFrame_, frameBuffer_, frameIndex_);
    lastFrameLength_ = frameIndex_;
    frameReceived_ = true;

}
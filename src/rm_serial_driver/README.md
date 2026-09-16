# rm_serial_driver

`rm_serial_driver` is the ROS 2 transport boundary between the Dart control board
and the rest of the system. It owns the serial port, receives fixed-size packets,
validates CRC16, and dispatches valid packets to registered event handlers.

## Runtime nodes

- `rm_serial_driver_node`: opens the configured serial port and dispatches
    `ReceivePacket` values.
- `virtual_serial_node`: generates the same `ReceivePacket` values from ROS
    parameters, so handlers can be exercised without hardware.

Both nodes load handlers registered in the shared library. The real node gets
packets from the serial port; the virtual node gets them from a 1 ms timer.

## Packet boundary

`ReceivePacket` is defined in `include/rm_serial_driver/packet.hpp`:

- frame header: `0x5A`
- task and control flags
- roll, pitch, yaw and bullet speed
- trailing CRC16 checksum

The serial node scans for the header, reads the remaining packet bytes, verifies
the checksum, and only then calls handlers. Invalid frames are discarded.

Serial parameters are `device_name`, `baud_rate`, `flow_control`, `parity`, and
`stop_bits`. The accepted string values are documented by the validation errors
in `SerialDriverNode::declareSerialParameters()`.

## Adding an event handler

1. Add a header under `include/rm_serial_driver/event_handler/`.
2. Derive from `rm_serial_driver::handler::BasicEventHandler` and implement
     `handleEvent(const ReceivePacket &)`.
3. Add the implementation to `EVENT_HANDLER_SOURCES` in `CMakeLists.txt`.
4. Register it at the end of the implementation with
     `REGISTER_EVENT_HANDLER(YourHandler)`.

Handlers should own one clearly defined ROS responsibility. They must not access
the serial port directly. The registry keeps the transport node independent of
feature-specific ROS topics and services.

## Development checks

Build this package with:

```bash
colcon build --packages-select rm_serial_driver
```

The package also enables the standard ROS 2 C++ lint and static-analysis tests
when `BUILD_TESTING` is enabled.
# rm_serial_driver

## 设计目标

`rm_serial_driver` 负责两条边界：

- 将控制板发来的串口数据转换为视觉系统可消费的 ROS 事件、状态和 TF。
- 将 planner/tracker 发来的 `GimbalCmd` 转换为串口下行数据并写入控制板。

串口硬件、收发线程、CRC、重连和发送时序由 `SerialDriverNode` 统一拥有。上层业务效果拆分到独立的 handler 和 writer 中，避免把所有 ROS 业务继续堆回串口主节点。

## 总体架构

```text

    SerialDriverNode(管理串口、事件处理器、串口写入器的生命周期)
      |      |
      |      v
      |  event_handler(对串口上传的数据做处理，每个子处理器负责一个方面(比如一个话题或者一个功能包))
      |
      v
    serial_writer(拥有串口写入权限，对下发数据处理并调用接口下发数据)
```

文件框架：

```text
rm_serial_driver/
├── CMakeLists.txt
├── package.xml
├── include/rm_serial_driver
│       ├── crc.hpp
│       ├── packet.hpp                     # 串口包定义
│       ├── rm_serial_driver_node.hpp
│       ├── event_handler/                 # 各类事件处理器（头文件）
│       │   ├── basic_event_handler.hpp    # 事件处理器基类
│       │   └── <各个子处理器头文件>
|       |
│       └── serial_writer/                 # 串口数据写入器（头文件）
│           ├── basic_serial_writer.hpp    # 串口写入器基类
│           └── <各个子写入器头文件>
└── src/
    ├── crc.cpp
    ├── rm_serial_driver_node.cpp
    ├── virtual_serial.cpp
    ├── event_handler/                     # 事件处理器实现文件夹
    # rm_serial_driver

    `rm_serial_driver` is the ROS 2 transport boundary between the Dart control board
    and the rest of the system. It owns the serial port, receives fixed-size packets,
    validates CRC16, and dispatches valid packets to registered event handlers.

    ## Runtime nodes

    - `rm_serial_driver_node`: opens the configured serial port and dispatches
        `ReceivePacket` values.
    - `virtual_serial_node`: generates the same `ReceivePacket` values from ROS
        parameters, so handlers can be exercised without hardware.

    Both nodes load handlers registered in the shared library. The real node gets
    packets from the serial port; the virtual node gets them from a 1 ms timer.

    ## Packet boundary

    `ReceivePacket` is defined in `include/rm_serial_driver/packet.hpp`:

    - frame header: `0x5A`
    - task and control flags
    - roll, pitch, yaw and bullet speed
    - trailing CRC16 checksum

    The serial node scans for the header, reads the remaining packet bytes, verifies
    the checksum, and only then calls handlers. Invalid frames are discarded.

    Serial parameters are `device_name`, `baud_rate`, `flow_control`, `parity`, and
    `stop_bits`.

    ## Adding an event handler

    1. Add a header under `include/rm_serial_driver/event_handler/`.
    2. Derive from `rm_serial_driver::handler::BasicEventHandler` and implement
         `handleEvent(const ReceivePacket &)`. 
    3. Add the implementation to `EVENT_HANDLER_SOURCES` in `CMakeLists.txt`.
    4. Register it with `REGISTER_EVENT_HANDLER(YourHandler)`.

    Handlers should own one clearly defined ROS responsibility and must not access
    the serial port directly. This keeps the transport node independent of feature
    specific ROS topics and services.

    ## Development checks

    ```bash
    colcon build --packages-select rm_serial_driver
    ```

    The package enables the standard ROS 2 C++ lint and static-analysis tests when
    `BUILD_TESTING` is enabled.


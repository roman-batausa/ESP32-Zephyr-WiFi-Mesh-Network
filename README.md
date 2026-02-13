# ESP32 Zephyr WiFi Mesh Network

A custom WiFi mesh networking implementation for ESP32 using Zephyr RTOS. This project creates a tree-topology mesh network where nodes communicate via UDP sockets, with support for mesh clock synchronization, static routing, and visual status indication via LEDs.

## Features

- **Tree-topology mesh networking** — Nodes organize in a parent-child hierarchy with a configurable topology map
- **Dual WiFi mode** — Each node runs both AP (Access Point) and STA (Station) interfaces simultaneously
- **Automatic routing** — Packets are forwarded through the mesh using a precomputed routing table
- **Clock synchronization** — All nodes synchronize their clocks to the root node for coordinated behavior
- **Node registration protocol** — Child nodes register with their parents before joining the mesh
- **LED status indicators** — Visual feedback for connection status and message reception
- **DHCPv4 server** — Each node's AP provides IP addresses to its children

## Hardware Requirements

- ESP32-DevKitC-WROOM (or compatible ESP32 board)
- 3 LEDs (optional, for status indication):
  - Green (GPIO 25) — Connected status
  - Yellow (GPIO 26) — Syncing / message received
  - Red (GPIO 27) — Disconnected status
- 2 Buttons (optional, GPIO 32/33)
- SH1106 OLED display (optional, I2C address 0x3C)
- MAX30101 sensor (optional, I2C address 0x57)

## Network Topology

The mesh uses a tree topology where:
- **Node 1** is the root node (connects to your WiFi router)
- Child nodes connect to their designated parent's AP
- Each non-leaf node runs its own AP for children to connect

Example topology defined in `main.c`:
```
Router (Internet)
    │
    └── Node 1 (Root)
           ├── Node 2
           └── Node 3
                  └── Node 4
```

## Configuration

### 1. Set the Node ID

Edit `src/mesh_config.h` and set the `NODE_ID` for each device before flashing:

```c
#define NODE_ID 1  // Change this for each node (1, 2, 3, 4, etc.)
```

### 2. Configure WiFi Credentials

For the root node to connect to your router, edit `src/mesh_config.h`:

```c
#define ROUTER_SSID  "YourWiFiName"     // Your router's SSID
#define ROUTER_PSK   "YourWiFiPassword" // Your router's password
```

### 3. Define the Topology

Edit the topology map in `src/main.c`:

```c
static const mesh_topology_t topology_map[] = {
    {1, 0}, // Node 1 is root (parent_id=0 means connects to router)
    {2, 1}, // Node 2's parent is Node 1
    {3, 1}, // Node 3's parent is Node 1
    {4, 3}, // Node 4's parent is Node 3
};
```

## Building and Flashing

### Prerequisites

- Zephyr RTOS SDK (v3.5+ recommended, v4.1 used for this project)
- ESP-IDF toolchain for Zephyr
- West build tool

### Build

```bash
west build -b esp32_devkitc_wroom/esp32/procpu
```

### Flash

```bash
west flash
```

### Monitor Serial Output

```bash
west espressif monitor
```

## Project Structure

```
wifi_mesh/
├── CMakeLists.txt
├── prj.conf                              # Zephyr project configuration
├── boards/
│   ├── esp32_devkitc_wroom_procpu.conf   # Board-specific Kconfig
│   └── esp32_devkitc_wroom_procpu.overlay # Device tree overlay
└── src/
    ├── main.c           # Application entry, LED threads, test packets
    ├── mesh_config.h    # Node ID, WiFi credentials, packet structures
    ├── zephyr_mesh.c    # Mesh networking implementation
    └── zephyr_mesh.h    # Mesh API declarations
```

## API Reference

### Initialization

```c
void zephyr_mesh_init(void);
```
Initialize the mesh module. Starts the UDP server thread and sets up WiFi event handlers.

```c
void zephyr_mesh_start(const mesh_topology_t *map, int map_size);
```
Start the mesh network with the given topology. Builds the routing table and begins scanning/connecting.

### Sending Data

```c
int zephyr_mesh_send_packet(uint8_t target, const char *payload);
```
Send a data packet to any node in the mesh. The routing table determines the path.

```c
int zephyr_mesh_send_to_child(uint8_t target, const char *payload);
```
Send directly to a connected child node (for nodes with an active AP).

### Status Queries

```c
bool zephyr_mesh_is_root(void);           // Is this the root node?
bool zephyr_mesh_is_connected(void);      // Connected to parent?
bool zephyr_mesh_is_registered(void);     // Registered with parent?
bool zephyr_mesh_is_clock_synced(void);   // Clock synchronized?
bool zephyr_mesh_received_a_message(void); // Message received since last check?
```

### Clock Synchronization

```c
int64_t zephyr_mesh_get_time_ms(void);
```
Get the synchronized mesh time in milliseconds. All nodes return the same value (within sync accuracy).

## Protocol Details

### Packet Types

| Type | Description |
|------|-------------|
| `MESH_PACKET_DATA` | Normal data payload |
| `MESH_PACKET_CLOCK_REQ` | Clock sync request (child → parent) |
| `MESH_PACKET_CLOCK_RESP` | Clock sync response (parent → child) |
| `MESH_PACKET_REGISTER` | Node registration request |
| `MESH_PACKET_REGISTER_ACK` | Registration acknowledgment |

### Network Addressing

- Each node creates an AP with SSID: `ESP32-ZephyrMesh-XX` (where XX is the node ID)
- AP IP addressing: `192.168.<NODE_ID>.1/24`
- Mesh communication port: `8080` (UDP)

### Connection Flow

1. Node boots and scans for parent's AP
2. Connects to parent via STA interface
3. Receives IP via DHCP from parent
4. Sends registration request to parent
5. Receives registration ACK (with optional initial clock sync)
6. Requests clock synchronization
7. Starts its own AP for child nodes
8. Ready to send/receive mesh packets

## LED Behavior

| LED | State | Meaning |
|-----|-------|---------|
| Yellow | Slow blink (500ms) | Waiting for clock sync |
| Yellow | Quick flash (200ms) | Message received |
| Green | Double blink (synced) | Connected to parent |
| Red | Double blink (synced) | Disconnected from parent |

All connected nodes blink in sync using the shared mesh time with a 4-second cycle period.

## Configuration Options

Key settings in `mesh_config.h`:

| Define | Default | Description |
|--------|---------|-------------|
| `MESH_PORT` | 8080 | UDP port for mesh communication |
| `MAX_CHILD_NODES` | 5 | Maximum children per node |
| `PACKET_TTL_MAX` | 10 | Maximum hops for packet forwarding |
| `MAX_ROUTING_ENTRIES` | 20 | Routing table size |
| `CLOCK_SYNC_RETRY_MS` | 1000 | Clock sync retry interval |

## Troubleshooting

**Node won't connect to parent:**
- Verify `NODE_ID` is unique and matches topology map
- Check that parent node is powered on and AP is running
- Ensure WiFi credentials are correct (for root node)

**Packets not being delivered:**
- Check serial logs for routing information
- Verify all intermediate nodes are connected
- Ensure `PACKET_TTL_MAX` is sufficient for your topology depth

**Clock sync failing:**
- Registration must complete before clock sync
- Check that parent node is reachable via UDP
- Monitor logs for "Clock synchronized!" message

## License

This project is provided as-is for educational and development purposes.

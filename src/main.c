#include "mesh_config.h"
#include "zephyr_mesh.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(MAIN);

// --- LED Timing Configuration ---
// All nodes will blink in sync using these shared timing parameters
#define LED_CYCLE_PERIOD_MS  4000  // Total LED cycle period (all nodes use the same)
#define CONN_BLINK_ON_MS     100   // Connection blink on duration
#define CONN_BLINK_OFF_MS    100   // Connection blink off duration
#define RECV_BLINK_MS        200   // Message received blink duration

// LED blink timing within the cycle (in milliseconds from cycle start)
#define BLINK1_START_MS      0
#define BLINK1_END_MS        100
#define BLINK2_START_MS      200
#define BLINK2_END_MS        300

static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(DT_ALIAS(led_g), gpios);
static const struct gpio_dt_spec led_y = GPIO_DT_SPEC_GET(DT_ALIAS(led_y), gpios);
static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(DT_ALIAS(led_r), gpios);

// --- Topology Definition ---
static const mesh_topology_t topology_map[] = {
    {1, 0}, // Node 1 is a root node (parent is router)
    {2, 1}, // Node 2's parent is Node 1
    {3, 1}, // Node 3's parent is Node 1
    {4, 3}, // Node 4's parent is Node 3
};

// --- Test Client Thread ---
static struct k_thread client_thread_data;
static k_tid_t client_tid;
K_THREAD_STACK_DEFINE(client_stack_area, 2048);

// --- LED Status Thread ---
static struct k_thread led_thread_data;
static k_tid_t led_tid;
K_THREAD_STACK_DEFINE(led_stack_area, 1024);

// --- Message received flash state ---
static volatile bool pending_recv_flash = false;

static void led_status_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("LED status thread started.");
    
    // Wait for clock sync before starting synchronized blinking
    LOG_INF("Waiting for clock synchronization...");
    while (!zephyr_mesh_is_clock_synced()) {
        // While waiting, show a slow amber blink to indicate "syncing"
        gpio_pin_set_dt(&led_y, 1);
        k_sleep(K_MSEC(500));
        gpio_pin_set_dt(&led_y, 0);
        k_sleep(K_MSEC(500));
    }
    LOG_INF("Clock synchronized! Starting synchronized LED blinking.");
    
    while (1) {
        // Get the synchronized mesh time
        int64_t mesh_time = zephyr_mesh_get_time_ms();
        
        // Calculate position within the LED cycle
        int64_t cycle_position = mesh_time % LED_CYCLE_PERIOD_MS;
        
        // Check for received messages - blink yellow LED
        if (zephyr_mesh_received_a_message()) {
            pending_recv_flash = true;
        }
        
        // Handle message received flash (takes priority, shown briefly)
        if (pending_recv_flash) {
            gpio_pin_set_dt(&led_y, 1);
            k_sleep(K_MSEC(RECV_BLINK_MS));
            gpio_pin_set_dt(&led_y, 0);
            pending_recv_flash = false;
            continue;  // Re-check timing after flash
        }
        
        // Connection status LED - synchronized double blink
        bool connected = zephyr_mesh_is_connected();
        
        // Determine if we should be ON based on cycle position
        bool should_be_on = false;
        if (cycle_position >= BLINK1_START_MS && cycle_position < BLINK1_END_MS) {
            should_be_on = true;
        } else if (cycle_position >= BLINK2_START_MS && cycle_position < BLINK2_END_MS) {
            should_be_on = true;
        }
        
        // Set LED state
        gpio_pin_set_dt(&led_g, (connected && should_be_on) ? 1 : 0);
        gpio_pin_set_dt(&led_r, (!connected && should_be_on) ? 1 : 0);
        
        // Calculate sleep time until next state change
        int64_t next_change;
        if (cycle_position < BLINK1_START_MS) {
            next_change = BLINK1_START_MS;
        } else if (cycle_position < BLINK1_END_MS) {
            next_change = BLINK1_END_MS;
        } else if (cycle_position < BLINK2_START_MS) {
            next_change = BLINK2_START_MS;
        } else if (cycle_position < BLINK2_END_MS) {
            next_change = BLINK2_END_MS;
        } else {
            // Wait until next cycle
            next_change = LED_CYCLE_PERIOD_MS;
        }
        
        int64_t sleep_time = next_change - cycle_position;
        if (sleep_time < 10) {
            sleep_time = 10;  // Minimum sleep to avoid busy spinning
        }
        
        k_sleep(K_MSEC(sleep_time));
    }
}

static void client_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Client (sender) thread started.");
    
    // Wait for clock sync before starting test packets
    while (!zephyr_mesh_is_clock_synced()) {
        k_sleep(K_MSEC(100));
    }
    LOG_INF("Clock synced, will start sending test packets.");
    
    while (1) {
        k_sleep(K_SECONDS(15));

        // --- TEST CASES ---
        // All child nodes send packets up to Node 1
        if (NODE_ID == 2) {
            LOG_INF("Node 2 sending test packet to Node 1...");
            zephyr_mesh_send_packet(1, "Hello from Node 2!");
        }
        if (NODE_ID == 3) {
            LOG_INF("Node 3 sending test packet to Node 1...");
            zephyr_mesh_send_packet(1, "Hello from Node 3!");
        }
        if (NODE_ID == 4) {
            LOG_INF("Node 4 sending test packet to Node 1...");
            zephyr_mesh_send_packet(1, "Hello from Node 4!");
            
            // Also test sending to Node 2 (requires routing through Node 3 -> Node 1 -> Node 2)
            k_sleep(K_SECONDS(2));
            LOG_INF("Node 4 sending test packet to Node 2...");
            zephyr_mesh_send_packet(2, "Hello from Node 4 to Node 2!");
        }

        // Root node sends replies back down
        if (NODE_ID == 1) {
            k_sleep(K_SECONDS(10)); // Stagger sends
            LOG_INF("Node 1 sending test packet to Node 2...");
            zephyr_mesh_send_packet(2, "Reply from Node 1 to Node 2!");
            
            k_sleep(K_SECONDS(2));
            LOG_INF("Node 1 sending test packet to Node 3...");
            zephyr_mesh_send_packet(3, "Reply from Node 1 to Node 3!");
            
            k_sleep(K_SECONDS(2));
            LOG_INF("Node 1 sending test packet to Node 4...");
            zephyr_mesh_send_packet(4, "Reply from Node 1 to Node 4!");
        }
        
        // Periodically log the mesh time for debugging
        LOG_INF("Current mesh time: %lld ms", zephyr_mesh_get_time_ms());
    }
}

// --- Main ---
int main(void)
{
    k_sleep(K_SECONDS(4));
    LOG_INF("--- ESP32 Zephyr Mesh Node %d ---", NODE_ID);
    
    // Initialize LEDs
    if (!gpio_is_ready_dt(&led_g) || !gpio_is_ready_dt(&led_y) || !gpio_is_ready_dt(&led_r)) {
        LOG_ERR("LED GPIO devices not ready!");
        return -1;
    }
    
    gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_y, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
    LOG_INF("LEDs initialized.");
    
    // Initialize the mesh module (starts server and sets up Wi-Fi)
    zephyr_mesh_init();
    
    // Configure and start the mesh (this builds routing table and begins scanning)
    zephyr_mesh_start(topology_map, ARRAY_SIZE(topology_map));

    // Start the LED status thread
    led_tid = k_thread_create(&led_thread_data,
                    led_stack_area,
                    K_THREAD_STACK_SIZEOF(led_stack_area),
                    led_status_thread,
                    NULL, NULL, NULL,
                    K_PRIO_PREEMPT(7),
                    0, K_NO_WAIT);

    // Start the test sender thread
    client_tid = k_thread_create(&client_thread_data,
                    client_stack_area,
                    K_THREAD_STACK_SIZEOF(client_stack_area),
                    client_thread,
                    NULL, NULL, NULL,
                    K_PRIO_PREEMPT(8),
                    0, K_NO_WAIT);

    return 0;
}

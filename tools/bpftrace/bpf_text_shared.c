#include <linux/blkdev.h>
#include <uapi/linux/ptrace.h>

/**
 * @brief Helper method to filter based on the specified inputString.
 * @param inputString The operation input string to check against the filter.
 * @return True if the specified inputString starts with the hard-coded FILTER_STRING; otherwise, false.
 */
static inline bool filter(char const* inputString)
{
    char needle[] = "FILTER_STRING"; ///< The FILTER STRING is replaced by python code.
    char haystack[sizeof(needle)] = {};
    bpf_probe_read(&haystack, sizeof(haystack), (void*)inputString);
    for (int i = 0; i < sizeof(needle) - 1; ++i) {
        if (needle[i] != haystack[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Contains the operation start data to trace.
 */

typedef struct
{
    u64 timestamp;
    int domain;
    char component_name[64];   ///< The component name
    char component_port[64];   ///< The port name
} EventQueueEvent;

struct start_data_t
{
    EventQueueEvent event;
    u64 start;        ///< Timestamp of the start operation (start timestamp).
};

/**
 * @brief Contains the operation start data.
 * key: the operation id.
 * value: The operation start latency data.
 */
BPF_HASH(start_hash, EventQueueEvent, struct start_data_t);

/**
 * @brief Reads the operation request arguments and stores the start data in the hash.
 * @param ctx The BPF context.
 */
int trace_operation_start(struct pt_regs* ctx)
{
    struct start_data_t start_data = {};
    EventQueueEvent ev;
    bpf_usdt_readarg_p(3, ctx, &start_data.component_name, sizeof(ev.component_name));
    bpf_usdt_readarg_p(4, ctx, &start_data.port_name, sizeof(ev.port_name));

    FILTER ///< Replaced by python code.

    bpf_usdt_readarg(1, ctx, &ev.domain);
    bpf_usdt_readarg(2, ctx, &ev.timestamp);

    start_data.event = event;

    start_data.start = bpf_ktime_get_ns();
    start_hash.update(&start_data.event, &start_data);
    return 0;
}

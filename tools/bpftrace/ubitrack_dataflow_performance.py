#!/usr/bin/env python
import argparse
from time import sleep
from sys import argv
import ctypes as ct
from bcc import BPF, USDT
import inspect
import os

# Parse command line arguments
parser = argparse.ArgumentParser(description="Trace the latency of an operation using usdt probes.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("-p", "--pid", type=int, help="The id of the process to trace.")
parser.add_argument("-f", "--filterstr", type=str, default="", help="The prefix filter for the operation input. If specified, only operations for which the input string starts with the filterstr are traced.")
parser.add_argument("-v", "--verbose", dest="verbose", action="store_true", help="If true, will output verbose logging information.")
parser.set_defaults(verbose=False)
args = parser.parse_args()
this_pid = int(args.pid)
this_filter = str(args.filterstr)

debugLevel=0
if args.verbose:
    debugLevel=4

# BPF program
bpf_text_shared = "%s/bpf_text_shared.c" % os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
bpf_text = open(bpf_text_shared, 'r').read()
bpf_text += """

/**
 * @brief Contains the latency data w.r.t. the complete operation from request to response.
 */
struct end_data_t
{
    u64 timestamp;
    int domain;
    char component_name[64];   ///< The component name
    char component_port[64];   ///< The port name

    u64 start;        ///< The start timestamp of the operation.
    u64 end;          ///< The end timestamp of the operation.
    u64 duration;     ///< The duration of the operation.
};

/**
 * The output buffer, which will be used to push the latency event data to user space.
 */
BPF_PERF_OUTPUT(operation_event);

/**
 * @brief Reads the operation response arguments, calculates the latency event data, and writes it to the user output buffer.
 * @param ctx The BPF context.
 */
int trace_operation_end(struct pt_regs* ctx)
{
    EventQueueEvent event;
    bpf_usdt_readarg(1, ctx, &event.domain);
    bpf_usdt_readarg(2, ctx, &event.timestamp);
    bpf_usdt_readarg_p(3, ctx, &event.component_name, sizeof(event.component_name));
    bpf_usdt_readarg_p(4, ctx, &event.port_name, sizeof(event.port_name));

    struct start_data_t* start_data = start_hash.lookup(&timestamp);
    if (0 == start_data) {
        return 0;
    }
    
    struct end_data_t end_data = {};
    end_data.event = event;
    
    end_data.end = bpf_ktime_get_ns();
    end_data.start = start_data->start;
    end_data.duration = end_data.end - end_data.start;
    end_data.domain = event.domain;
    end_data.timestamp = event.timestamp;
    __builtin_memcpy(&end_data.component_name, event->component_name, sizeof(event.component_name));
    __builtin_memcpy(&end_data.port_name, event->port_name, sizeof(event.port_name));

    start_hash.delete(&event);

    operation_event.perf_submit(ctx, &end_data, sizeof(end_data));
    return 0;
}
"""

bpf_text = bpf_text.replace("FILTER_STRING", this_filter)
if this_filter:
    bpf_text = bpf_text.replace("FILTER", "if (!filter(start_data.event.component_name)) { return 0; }")
else:
    bpf_text = bpf_text.replace("FILTER", "")

# Create USDT context
print("Attaching probes to pid %d" % this_pid)
usdt_ctx = USDT(pid=this_pid)
usdt_ctx.enable_probe(probe="eventqueue_dispatch_begin", fn_name="trace_operation_start")
usdt_ctx.enable_probe(probe="eventqueue_dispatch_end", fn_name="trace_operation_end")

# Create BPF context, load BPF program
bpf_ctx = BPF(text=bpf_text, usdt_contexts=[usdt_ctx], debug=debugLevel)

# Define latency event and print function
class OperationEventData(ct.Structure):
    _fields_ = [("domain", ct.c_int),
                ("timestamp", ct.c_ulonglong)
                ("component_name", ct.c_char * 64),
                ("port_name", ct.c_char * 64),
                ("start", ct.c_ulonglong),
                ("end", ct.c_ulonglong),
                ("duration", ct.c_ulonglong)]

start = 0
def print_event(cpu, data, size):
    global start
    event = ct.cast(data, ct.POINTER(OperationEventData)).contents
    if start == 0:
        start = event.start
    time_s = (float(event.start - start)) / 1000000000
    latency = (float(event.duration) / 1000)
    print("%-18.9f %-10d %-32s %-32s %16d %16d %16d" % (time_s, event.timestamp, event.componentname, event.port_name, event.start, event.end, latency))

# Print header
print("Tracing... Hit Ctrl-C to end.")
print("%-18s %-10s %-32s %-32s %16s %16s %16s" % ("time(s)", "ts", "component", "port", "start (ns)", "end (ns)", "duration (us)"))

# Output latency events
bpf_ctx["operation_event"].open_perf_buffer(print_event)
while 1:
    bpf_ctx.perf_buffer_poll()
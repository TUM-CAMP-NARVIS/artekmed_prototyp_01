#!/usr/bin/env python
import argparse
from time import sleep
from sys import argv
import ctypes as ct
from bcc import BPF, USDT
import inspect
import os

# Parse command line arguments
parser = argparse.ArgumentParser(description="Trace rendering performance.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("-p", "--pid", type=int, help="The id of the process to trace.")
parser.add_argument("-v", "--verbose", dest="verbose", action="store_true", help="If true, will output verbose logging information.")
parser.set_defaults(verbose=False)
args = parser.parse_args()
this_pid = int(args.pid)

debugLevel=0
if args.verbose:
    debugLevel=4

# BPF program
bpf_text_shared = "%s/bpf_text_artekmed_shared.c" % os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
bpf_text = open(bpf_text_shared, 'r').read()
bpf_text += """

/**
 * @brief Contains the latency data w.r.t. the complete operation from request to response.
 */
struct end_data_t
{
    u64 count;
    u64 start;        ///< The start timestamp of the operation.
    u64 end;          ///< The end timestamp of the operation.
    u64 duration;     ///< The duration of the operation.
};

/**
 * The output buffer, which will be used to push the latency event data to user space.
 */
BPF_PERF_OUTPUT(visualizer_render_event);

/**
 * @brief Reads the operation response arguments, calculates the latency event data, and writes it to the user output buffer.
 * @param ctx The BPF context.
 */
int trace_visualizer_render_end(struct pt_regs* ctx)
{
    u64 count;
    bpf_usdt_readarg(1, ctx, &count);

    struct start_data_t* start_data = start_hash.lookup(&count);
    if (0 == start_data) {
        return 0;
    }
    
    struct end_data_t end_data = {};
    end_data.end = bpf_ktime_get_ns();
    end_data.start = start_data->start;
    end_data.duration = end_data.end - end_data.start;
    end_data.count = count;

    start_hash.delete(&count);

    visualizer_render_event.perf_submit(ctx, &end_data, sizeof(end_data));
    return 0;
}
"""

bpf_text = bpf_text.replace("FILTER", "")

# Create USDT context
print("Attaching probes to pid %d" % this_pid)
usdt_ctx = USDT(pid=this_pid)
usdt_ctx.enable_probe(probe="visualizer_render_begin", fn_name="trace_visualizer_render_begin")
usdt_ctx.enable_probe(probe="visualizer_render_end", fn_name="trace_visualizer_render_end")

# Create BPF context, load BPF program
bpf_ctx = BPF(text=bpf_text, usdt_contexts=[usdt_ctx], debug=debugLevel)

# Define latency event and print function
class RenderEventData(ct.Structure):
    _fields_ = [("count", ct.c_int),
                ("start", ct.c_ulonglong),
                ("end", ct.c_ulonglong),
                ("duration", ct.c_ulonglong)]

start = 0
def print_event(cpu, data, size):
    global start
    event = ct.cast(data, ct.POINTER(RenderEventData)).contents
    if start == 0:
        start = event.start
    time_s = (float(event.start - start)) / 1000000000
    latency = (float(event.duration) / 1000)
    print("%-18.9f %-10d %16d %16d %16d" % (time_s, event.count, event.start, event.end, latency))

# Print header
print("Tracing... Hit Ctrl-C to end.")
print("%-18s %-10s %16s %16s %16s" % ("time(s)", "count", "start (ns)", "end (ns)", "duration (us)"))

# Output latency events
bpf_ctx["visualizer_render_event"].open_perf_buffer(print_event)
while 1:
    bpf_ctx.perf_buffer_poll()
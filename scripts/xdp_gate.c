// XDP program: drop UDP packets to port 7502 based on configurable drop rate
// Drops happen at NIC driver level — invisible to Wireshark/tcpdump
#include <linux/bpf.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/in.h>
#include <bpf/bpf_helpers.h>

// Map: key=0 → drop_pct (0-100), key=1 → pkts_passed, key=2 → pkts_dropped
struct {
    __uint(type, BPF_MAP_TYPE_ARRAY);
    __uint(max_entries, 4);
    __type(key, __u32);
    __type(value, __u64);
} gate_map SEC(".maps");

SEC("xdp")
int xdp_lidar_gate(struct xdp_md *ctx) {
    void *data = (void *)(long)ctx->data;
    void *data_end = (void *)(long)ctx->data_end;

    // Parse Ethernet header
    struct ethhdr *eth = data;
    if ((void *)(eth + 1) > data_end)
        return XDP_PASS;
    if (eth->h_proto != __constant_htons(ETH_P_IP))
        return XDP_PASS;

    // Parse IP header
    struct iphdr *ip = (void *)(eth + 1);
    if ((void *)(ip + 1) > data_end)
        return XDP_PASS;
    if (ip->protocol != IPPROTO_UDP)
        return XDP_PASS;

    // Parse UDP header
    struct udphdr *udp = (void *)ip + (ip->ihl * 4);
    if ((void *)(udp + 1) > data_end)
        return XDP_PASS;

    // Check destination port (7502 = 0x1D4E, network byte order)
    if (udp->dest != __constant_htons(7502))
        return XDP_PASS;

    // This is a LiDAR packet — check gate
    __u32 key_pct = 0;
    __u64 *drop_pct = bpf_map_lookup_elem(&gate_map, &key_pct);
    if (!drop_pct || *drop_pct == 0) {
        // Gate open — pass all
        __u32 key_pass = 1;
        __u64 *pass_cnt = bpf_map_lookup_elem(&gate_map, &key_pass);
        if (pass_cnt) __sync_fetch_and_add(pass_cnt, 1);
        return XDP_PASS;
    }

    // Random drop based on percentage
    __u32 rand = bpf_get_prandom_u32() % 100;
    if (rand < *drop_pct) {
        // DROP — NIC driver level, invisible to Wireshark
        __u32 key_drop = 2;
        __u64 *drop_cnt = bpf_map_lookup_elem(&gate_map, &key_drop);
        if (drop_cnt) __sync_fetch_and_add(drop_cnt, 1);
        return XDP_DROP;
    }

    // PASS
    __u32 key_pass = 1;
    __u64 *pass_cnt = bpf_map_lookup_elem(&gate_map, &key_pass);
    if (pass_cnt) __sync_fetch_and_add(pass_cnt, 1);
    return XDP_PASS;
}

char _license[] SEC("license") = "GPL";

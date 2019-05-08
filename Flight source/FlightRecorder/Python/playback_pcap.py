#!/usr/bin/python
# Plays back a .pcap file's UDP packets to a specified destination address.
# This script assumes that the UDP packets contained in the capture follow
# a messaging protocol where the message ID is found in bytes 0 and 1.
#
# Note that Wireshark now defaults to .pcapng.  At the time of writing, there
# weren't python libraries to perform basic UDP protocol dissection with this
# format.  Thus, this script only works with the older .pcap format.

print("Importing...")
import socket
import struct
import time
import math
import sys
try:
    import pcapfile
    from pcapfile import savefile
except ImportError:
    print("This script requires the pcapfile library.  Please run:")
    print("sudo pip install pypcapfile")
    print("or see the installation instructions at:")
    print("https://pypi.python.org/pypi/pypcapfile")
    exit()

print("done.")

# Network settings
UDP_IP = "255.255.255.255"
# UDP_IP = "10.11.34.240"
UDP_PORT = 6599

# 2.0 is 2x fast forward, 0.5 is half speed
TIME_SCALE_FACTOR = 1.0

filename = ""
if len(sys.argv) != 2:
    print("Usage: python playback_pcap.py file.pcap")
    exit()
else:
    filename = sys.argv[1]

capture = savefile.load_savefile(open(filename, 'rb'), layers = 3)

# Open outbound socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


def send_packet(data):
    bytes = [data[i:i+2] for i in range(0, len(data), 2)]
    nums = [int(B, 16) for B in bytes]
    chars = [chr(B) for B in nums]
    str = ''.join(chars)
    sock.sendto(str, (UDP_IP, UDP_PORT))

# Only playback packets matching these properties.
# Remove a line to stop filtering on that property.
filter = {
    # 'udp_src_port' : 1,
    'udp_dst_port' : 6599,
    # 'ip_src'       : "192.168.1.1",
    'ip_dst'       : "255.255.255.255",
    # Message ID must match one of these.
    # MID is assumed to be the 3rd and 4th byte of UDP payload, LSB-first.
    'mid': [0x0100, 0x0101],
}

# Read and replay packets
prev_fragment_payload = ''
last_timestamp = None
for packet in capture.packets:
    # Verify UDP
    timestamp = packet.timestamp + 0.001 * packet.timestamp_ms
    # IP layer
    ip_packet = packet.packet.payload
    if isinstance(ip_packet, pcapfile.protocols.network.ip.IP):
        if filter.has_key('ip_src'):
            if ip_packet.src != filter['ip_src']:
                print("IP src mismatch: found %s, expected %s"%(ip_packet.src, filter['ip_src']))
                continue
        if filter.has_key('ip_dst'):
            if ip_packet.dst != filter['ip_dst']:
                print("IP dst mismatch: found %s, expected %s"%(ip_packet.dst, filter['ip_dst']))
                continue
        
        # UDP layer
        udp_packet = ip_packet.payload
        if isinstance(udp_packet, pcapfile.protocols.transport.udp.UDP):
            # Deal with fragmentation
            if ip_packet.flags == 1: # More fragments follow
                if len(prev_fragment_payload) == 0:
                    orig_pkt_udp_src_port = udp_packet.src_port
                    orig_pkt_udp_dst_port = udp_packet.dst_port
                else:
                    pass # Preserve the last iteration's values
                prev_fragment_payload = prev_fragment_payload + udp_packet.payload
            elif len(prev_fragment_payload) == 0:
                orig_pkt_udp_src_port = udp_packet.src_port
                orig_pkt_udp_dst_port = udp_packet.dst_port
            
            if filter.has_key('udp_src_port'):
                if orig_pkt_udp_src_port != filter['udp_src_port']:
                    print("UDP src port mismatch: found %d, expected %d"%(orig_pkt_udp_src_port, filter['udp_src_port']))
                    continue
            if filter.has_key('udp_dst_port'):
                if orig_pkt_udp_dst_port != filter['udp_dst_port']:
                    print("UDP dst port mismatch: found %d, expected %d"%(orig_pkt_udp_dst_port, filter['udp_dst_port']))
                    continue
            
            if ip_packet.flags != 1:
                # In hex string for some reason
                payload = prev_fragment_payload + udp_packet.payload
                prev_fragment_payload = ''
                
                # Messaging protocol layer
                if len(payload) >= 8: # 4B
                    msg_id = int(payload[0:2], 16) + 0x100 * int(payload[2:4], 16)
                    mid_matches = True
                    if filter.has_key('mid'):
                        mid_matches = False
                        for good_mid in filter['mid']:
                            mid_matches = mid_matches or (good_mid == msg_id)
                    if mid_matches:
                        if last_timestamp == None:
                            delta = 0
                        else:
                            delta = timestamp - last_timestamp
                        last_timestamp = timestamp
                        delta /= TIME_SCALE_FACTOR
                        print("Sleeping %fs to mimic timeline"%delta)
                        time.sleep(delta)
                        
                        print("Sending packet with MID 0x%04X"%msg_id)
                        send_packet(payload)
                    else:
                        print("MID 0x%04X not in whitelist."%msg_id)
                else:
                    print("Packet too short to have a message ID")
            else:
                print("Fragment; not yet sending.")
        else:
            print("Rejected non UDP packet of class %s"%str(udp_packet.__class__))
    else:
        print("Rejected non IP packet of class %s"%str(ip_packet.__class__))
        
        
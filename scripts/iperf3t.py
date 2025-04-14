
import iperf3
import datetime
from enum import Enum

class Direction(Enum):
    UPLINK = 0
    DOWNLINK = 1

class Iperf3Tester:
    def __init__(self):
        self.iperf3_client = iperf3.Client()

    def run_iperf3_client(self, server, direction, target_throughput=None, port=5201, duration=30, num_streams=5):
        """Runs an iperf3 client with multiple parallel streams (-P option)."""
        client = iperf3.Client()
        client.server_hostname = server
        client.port = port
        client.duration = duration
        client.num_streams = num_streams  # Equivalent to -P 5 in CLI
        client.protocol = 'tcp'  # Change to 'udp' if needed
        if direction == Direction.DOWNLINK:
            client.reverse = True
        if target_throughput is not None:
            if target_throughput == 0:
                client.bandwidth = 1
            else:
                client.bandwidth = int(target_throughput * 1000**2 / num_streams)

        print(f"Starting iperf3 test to {server}:{port} for {duration} seconds with target throughput {target_throughput} Mbps and {num_streams} parallel streams...")

        result = client.run()

        if result.error:
            print(f"Error: {result.error}")
        else:
            print(f"Sent: {result.sent_Mbps} Mbps, Received: {result.received_Mbps} Mbps")
        return result

# iperf3_tester = Iperf3Tester()
# iperf3_tester.run_iperf3_client("10.1.101.18", Direction.DOWNLINK, 10)

import datetime
from enum import Enum
import json
import uuid
import iperf3
from clickhouse_driver import Client

class Direction(Enum):
    UPLINK = 0
    DOWNLINK = 1

class TargetThroughput(Enum):
    NO_THROUGHPUT = 0
    ONE_MBPS_TIMES = 1
    TWO_MBPS_TIMES = 2
    FOUR_MBPS_TIMES = 4
    EIGHT_MBPS_TIMES = 8
    SIXTEEN_MBPS_TIMES = 16
    THIRTY_TWO_MBPS_TIMES = 32
    SIXTY_FOUR_MBPS_TIMES = 64
    NO_LIMIT = -1

class TestingSenario:
    def __init__(self, direction, target_throughput):
        self.direction = direction
        self.target_throughput = target_throughput

        if target_throughput == TargetThroughput.NO_THROUGHPUT:
            self.bandwidth = 1
        elif target_throughput == TargetThroughput.ONE_MBPS_TIMES:
            self.bandwidth = 1 * 1000**2
        elif target_throughput == TargetThroughput.TWO_MBPS_TIMES:
            self.bandwidth = 2 * 1000**2
        elif target_throughput == TargetThroughput.FOUR_MBPS_TIMES:
            self.bandwidth = 4 * 1000**2
        elif target_throughput == TargetThroughput.EIGHT_MBPS_TIMES:
            self.bandwidth  = 8 * 1000**2
        elif target_throughput == TargetThroughput.SIXTEEN_MBPS_TIMES:
            self.bandwidth  = 16 * 1000**2
        elif target_throughput == TargetThroughput.THIRTY_TWO_MBPS_TIMES:
            self.bandwidth = 32 * 1000**2
        elif target_throughput == TargetThroughput.SIXTY_FOUR_MBPS_TIMES:
            self.bandwidth = 64 * 1000**2
        else:
            self.bandwidth = -1

TestingSenarios = [
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.NO_THROUGHPUT),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.ONE_MBPS_TIMES),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.TWO_MBPS_TIMES),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.FOUR_MBPS_TIMES),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.EIGHT_MBPS_TIMES),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.SIXTEEN_MBPS_TIMES),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.THIRTY_TWO_MBPS_TIMES),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.NO_LIMIT),
    TestingSenario(direction=Direction.DOWNLINK, target_throughput=TargetThroughput.NO_THROUGHPUT),

    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.NO_THROUGHPUT),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.ONE_MBPS_TIMES),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.TWO_MBPS_TIMES),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.FOUR_MBPS_TIMES),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.EIGHT_MBPS_TIMES),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.SIXTEEN_MBPS_TIMES),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.THIRTY_TWO_MBPS_TIMES),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.NO_LIMIT),
    TestingSenario(direction=Direction.UPLINK, target_throughput=TargetThroughput.NO_THROUGHPUT),
]

class Iperf3Tester:
    def __init__(self):
        # Connect to ClickHouse server
        self.clickhouse = Client(host='172.27.2.22', port=9000)
        result = self.clickhouse.execute('SELECT version()')
        print(f"ClickHouse version: {result[0][0]}")

        # self.clickhouse.execute("""
        # CREATE TABLE IF NOT EXISTS iperf3db (
        #     timestamp DateTime64,
        #     seconds Float64,
        #     direction UInt8,
        #     bytes_transferred UInt64,
        #     bits_per_second Float64,
        #     retransmits UInt32,
        #     snd_cwnd Float64,
        #     rtt Float64,
        #     rttvar Float64,
        #     pmtu Float64
        # ) ENGINE = MergeTree()
        # ORDER BY (timestamp)
        # """)

        self.clickhouse.execute("""
        CREATE TABLE IF NOT EXISTS iperf3db (
            test_id String,
            timestamp DateTime64,
            seconds Float64,
            direction UInt8,
            bytes_transferred UInt64,
            target_throughput_mbps Int64,
            throughput_mbps Float64,
        ) ENGINE = MergeTree()
        ORDER BY (timestamp)
        """)

    def run_iperf3_client(self, server, senario, port=5201, duration=30, num_streams=5):
        """Runs an iperf3 client with multiple parallel streams (-P option)."""
        client = iperf3.Client()
        client.server_hostname = server
        client.port = port
        client.duration = duration
        client.num_streams = num_streams  # Equivalent to -P 5 in CLI
        client.protocol = 'tcp'  # Change to 'udp' if needed
        if senario.direction == Direction.DOWNLINK:
            client.reverse = True
        if senario.target_throughput != TargetThroughput.NO_LIMIT:
            client.bandwidth = int(senario.bandwidth)

        print(f"Starting iperf3 test to {server}:{port} for {duration} seconds with {num_streams} parallel streams...")

        result = client.run()

        if result.error:
            print(f"Error: {result.error}")
        else:
            print(f"Sent: {result.sent_Mbps} Mbps, Received: {result.received_Mbps} Mbps")
        return result

    def publish_result_to_clickhouse(self, result):
        # Extract test metadata
        test_id = result['test_id']
        direction = result['direction']
        iperf3_result = result['iperf3_result']

        # Parse timestamp from timesecs with UTC timezone
        timestamp = datetime.datetime.fromtimestamp(
            iperf3_result['start']['timestamp']['timesecs'],
            tz=datetime.timezone.utc
        )
        # Convert timestamp from UTC to TAI (International Atomic Time)
        timestamp = timestamp + datetime.timedelta(seconds=37)

        # Prepare data for insertion (per-stream, per-interval)
        rows = []
        for interval in iperf3_result['intervals']:
            interval_sum = interval['sum']
            sum_timstamp = timestamp + datetime.timedelta(seconds=interval_sum['start'])
            sum_second = interval_sum['seconds']
            sum_direction = direction
            sum_target_throughput = result['target_throughput']
            sum_bytes = interval_sum['bytes']
            sum_throughput_mbps = interval_sum['bits_per_second'] / 10**6
            # sum_retransmits = interval_sum['retransmits']
            # sum_snd_cwnd = sum(stream['snd_cwnd'] for stream in interval['streams']) / len(interval['streams'])
            # sum_rtt = sum(stream['rtt'] for stream in interval['streams']) / len(interval['streams'])
            # sum_rttvar = sum(stream['rttvar'] for stream in interval['streams']) / len(interval['streams'])
            # sum_pmtu = sum(stream['pmtu'] for stream in interval['streams']) / len(interval['streams'])
            row = (
                test_id,
                sum_timstamp,
                sum_second,
                sum_direction,
                sum_bytes,
                sum_target_throughput,
                sum_throughput_mbps,
                # sum_retransmits,
                # sum_snd_cwnd,
                # sum_rtt,
                # sum_rttvar,
                # sum_pmtu
            )
            rows.append(row)

        self.clickhouse.execute(
            'INSERT INTO iperf3db (test_id, timestamp, seconds, direction, bytes_transferred, target_throughput_mbps, throughput_mbps) VALUES',
            rows
        )
        print(f"Inserted {len(rows)} rows into ClickHouse")

    def run(self, senarios):
        test_id = uuid.uuid4().hex
        for senario in senarios:
            print(f"*** Running iperf3 test in {senario.direction.name} direction with target throughput {senario.target_throughput}")
            iperf3_result = self.run_iperf3_client("10.1.101.18", senario)
            result = {
                "test_id": test_id,
                "direction": senario.direction.value,
                "target_throughput": senario.target_throughput == TargetThroughput.NO_LIMIT and -1 or senario.target_throughput.value * 5,
                "iperf3_result": iperf3_result.json
            }
            with open('iperf3_result_downlink.json', 'w') as f:
                json.dump(result, f, indent=4)
            self.publish_result_to_clickhouse(result)

if __name__ == "__main__":
    iperf3_test = Iperf3Tester()
    iperf3_test.run(senarios=TestingSenarios)

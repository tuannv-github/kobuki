from iperf3t import *
from mtrx import *
from clickhouse_driver import Client
import uuid
import time
import numpy as np

iperf3_tester = Iperf3Tester()

mtrx = MTRX()
mtrx.get_output_frequency()
mtrx.set_output_frequency(3450)
mtrx.set_output_state(0)
time.sleep(5)

TEST_ID = uuid.uuid4().hex

# noise_level = [-100]
# noise_level = [-100, -50, -30, -25, -20, -15, -14, -13, -12, -11.9, -11.8, -11.7, -11.6, -11.5, -11.4, -11.3, -11.2, -11.1, -11]
# noise_level = [-12, -11.9, -11.8, -11.7, -11.6, -11.5, -11.4, -11.3, -11.2, -11.1, -11]
X_TIMES = 5
noise_level = [x/X_TIMES for x in range(-12*X_TIMES, -0*X_TIMES, int(0.2*X_TIMES))]

class Tester:
    def __init__(self):
        # Connect to ClickHouse server
        self.clickhouse = Client(host='172.27.2.22', port=9000)
        result = self.clickhouse.execute('SELECT version()')
        print(f"ClickHouse version: {result[0][0]}")

        self.clickhouse.execute("""
        CREATE TABLE IF NOT EXISTS wgn (
            test_id String,
            timestamp DateTime64,
            noise_level Float64,
            direction UInt8,
            target_throughput_mbps Int64,
            throughput_mbps Float64,
        ) ENGINE = MergeTree()
        ORDER BY (timestamp)
        """)
    
    def publish_result_to_clickhouse(self, result):
        test_id = result["test_id"]
        noise_level = result["noise_level"]
        direction = result["direction"]
        target_throughput_mbps = result["target_throughput"]
        iperf3_result = result["iperf3_result"]

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
            current_timestamp = timestamp + datetime.timedelta(seconds=interval_sum['start'])
            current_throughput_mbps = interval_sum['bits_per_second'] / 10**6
            # sum_retransmits = interval_sum['retransmits']
            # sum_snd_cwnd = sum(stream['snd_cwnd'] for stream in interval['streams']) / len(interval['streams'])
            # sum_rtt = sum(stream['rtt'] for stream in interval['streams']) / len(interval['streams'])
            # sum_rttvar = sum(stream['rttvar'] for stream in interval['streams']) / len(interval['streams'])
            # sum_pmtu = sum(stream['pmtu'] for stream in interval['streams']) / len(interval['streams'])
            row = (
                test_id,
                current_timestamp,
                noise_level,
                direction,
                target_throughput_mbps, 
                current_throughput_mbps
            )
            rows.append(row)

        self.clickhouse.execute(
            'INSERT INTO wgn (test_id, timestamp, noise_level, direction, target_throughput_mbps, throughput_mbps) VALUES',
            rows
        )
        print(f"Inserted {len(rows)} rows into ClickHouse")

tester = Tester()
for noise in noise_level:
    print(f"Noise level: {noise}")
    mtrx.set_output_level(noise)
    mtrx.set_output_state(1)
    time.sleep(5)

    # iperf3_result = iperf3_tester.run_iperf3_client("10.1.101.18", Direction.DOWNLINK)
    # result = {
    #     "test_id": TEST_ID,
    #     "noise_level": noise,
    #     "direction": Direction.DOWNLINK.value,
    #     "target_throughput": -1,
    #     "iperf3_result": iperf3_result.json
    # }
    # tester.publish_result_to_clickhouse(result)
    # time.sleep(10)
    
    iperf3_result = iperf3_tester.run_iperf3_client("10.1.101.18", Direction.UPLINK)
    result = {
        "test_id": TEST_ID,
        "noise_level": noise,
        "direction": Direction.UPLINK.value,
        "target_throughput": -1,
        "iperf3_result": iperf3_result.json
    }
    tester.publish_result_to_clickhouse(result)
    time.sleep(10)

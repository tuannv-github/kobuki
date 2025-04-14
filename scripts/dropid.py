#!python

from clickhouse_driver import Client
import sys

# Define your ClickHouse connection parameters
HOST = "172.27.2.22"
PORT = 9000  # Assuming native protocol port works based on prior troubleshooting
TABLE = "iperf3db"

# Get DROP_ID from command-line argument
if len(sys.argv) < 2:
    print("Error: Please provide a DROP_ID as an argument")
    sys.exit(1)
DROP_ID = sys.argv[1]

# Establish a connection
client = Client(host=HOST, port=PORT)

# Execute the DELETE query
query = f"ALTER TABLE {TABLE} DELETE WHERE test_id = '{DROP_ID}'"
client.execute(query)

print(f"Deleted all rows where test_id = '{DROP_ID}'")

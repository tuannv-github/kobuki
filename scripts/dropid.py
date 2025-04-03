from clickhouse_driver import Client

# Define your ClickHouse connection parameters
HOST = "172.27.2.22"
PORT = 9000  # Assuming native protocol port works based on prior troubleshooting
TABLE = "iperf3db"
DROP_ID = "59823ab3d1974284bb81c388b53fc804"

# Establish a connection
client = Client(host=HOST, port=PORT)

# Execute the DELETE query
query = f"ALTER TABLE {TABLE} DELETE WHERE test_id = '{DROP_ID}'"
client.execute(query)

print(f"Deleted all rows where test_id = '{DROP_ID}'")
import serial
import json

ser = serial.Serial(
    "/dev/ttyACM0",
    115200,
    timeout=1
)

last_d = None
last_p = None

d_count = 0
p_count = 0
pair_count = 0

last_pair = None

while True:

    raw = ser.readline()

    if not raw:
        continue

    line = raw.decode(
        "utf-8",
        errors="ignore"
    ).strip()

    json_start = line.find("{")

    if json_start == -1:
        continue

    try:
        data = json.loads(
            line[json_start:]
        )

        twr = data["TWR"]

        d = twr["D"]
        p = twr["P"]

    except (
        json.JSONDecodeError,
        KeyError,
        TypeError
    ):
        continue


    # Distance repeat counter
    if d == last_d:
        d_count += 1
    else:
        d_count = 1
        last_d = d


    # Angle repeat counter
    if p == last_p:
        p_count += 1
    else:
        p_count = 1
        last_p = p


    # Combined counter
    pair = (d, p)

    if pair == last_pair:
        pair_count += 1
    else:
        pair_count = 1
        last_pair = pair


    print(
        f"D={d:5}  "
        f"P={p:5}  |  "
        f"D repeat={d_count:3}  "
        f"P repeat={p_count:3}  "
        f"PAIR repeat={pair_count:3}"
    )
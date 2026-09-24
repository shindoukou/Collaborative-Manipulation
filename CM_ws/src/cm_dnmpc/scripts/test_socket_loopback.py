#!/usr/bin/env python3
import threading
import time

from cm_dnmpc_transport import LocalDuplexPredictionSocket


def main():
    left_port = 55995
    right_port = 55996
    left = LocalDuplexPredictionSocket(
        "127.0.0.1", "127.0.0.1", left_port, right_port, 1.0, 0.05)
    right = LocalDuplexPredictionSocket(
        "127.0.0.1", "127.0.0.1", right_port, left_port, 1.0, 0.05)
    output = {}

    def left_run():
        left.wait_connected(lambda: False)
        output["left"] = left.exchange({"side": "left", "sequence": 1})

    thread = threading.Thread(target=left_run)
    thread.start()
    time.sleep(0.05)
    right.wait_connected(lambda: False)
    output["right"] = right.exchange({"side": "right", "sequence": 2})
    thread.join(2.0)
    left.close()
    right.close()
    if output.get("left", {}).get("sequence") != 2:
        raise RuntimeError("left exchange failed")
    if output.get("right", {}).get("sequence") != 1:
        raise RuntimeError("right exchange failed")
    print("LOCAL_DUPLEX_TCP_LOOPBACK_OK", output)


if __name__ == "__main__":
    main()

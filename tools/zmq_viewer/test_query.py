#!/usr/bin/env python3
"""
Test script to verify parameter query functionality
"""

import zmq
import json
import sys

def test_query(host="192.168.100.6", port=5570):
    """Test querying parameters from device."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 3000)

    try:
        sock.connect(f"tcp://{host}:{port}")
        print(f"Connected to {host}:{port}")

        # Send query command
        cmd = {"query": True}
        print(f"\nSending: {json.dumps(cmd)}")
        sock.send_string(json.dumps(cmd))

        # Receive response
        reply = sock.recv_string()
        print(f"\nReceived: {reply}")

        # Parse and display
        resp = json.loads(reply)
        if resp.get("status") == "ok":
            print("\n✓ Query successful!")
            print("\nCamera Parameters:")
            print("-" * 60)
            for cam in resp.get("cameras", []):
                print(f"Camera {cam['index']}:")
                print(f"  Auto Exposure: {cam['auto_exposure']}")
                print(f"  Exposure:      {cam['exposure_us']} us")
                print(f"  Gain:          {cam['analogue_gain']}")
                print(f"  JPEG Quality:  {cam['jpeg_quality']}")
                print()
        else:
            print(f"✗ Query failed: {resp.get('message', 'Unknown error')}")
            return False

    except zmq.Again:
        print("✗ Timeout - device not responding")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False
    finally:
        sock.close()
        ctx.term()

    return True

if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.100.6"
    success = test_query(host)
    sys.exit(0 if success else 1)

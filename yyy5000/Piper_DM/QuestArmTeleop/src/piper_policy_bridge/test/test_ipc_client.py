import json
import unittest

import numpy as np

from piper_policy_bridge.ipc_client import MAGIC, PolicyClient


class PolicyClientTest(unittest.TestCase):
    def test_reset_requires_server_acknowledgement(self):
        client = PolicyClient("ipc:///unused")
        client._exchange = lambda parts: [MAGIC, b'{"reset":true}', b""]

        self.assertIsNone(client.reset(3))

        client._exchange = lambda parts: [
            MAGIC,
            b'{"error":"reset failed"}',
            b"",
        ]
        with self.assertRaisesRegex(RuntimeError, "reset failed"):
            client.reset(4)

    def test_multiview_request_contains_both_images_and_timestamps(self):
        client = PolicyClient("ipc:///unused")
        captured = []
        action = np.zeros((4, 7), dtype=np.float32)

        def exchange(parts):
            captured.extend(parts)
            return [
                MAGIC,
                json.dumps(
                    {
                        "request_id": 7,
                        "shape": [4, 7],
                        "observation_timestamp": 10.005,
                        "policy_type": "act",
                        "execution_steps": 1,
                        "temporal_ensemble_coeff": 0.01,
                        "period_s": 0.1,
                    }
                ).encode(),
                action.tobytes(),
            ]

        client._exchange = exchange
        state = np.zeros(14, dtype=np.float32)
        wrist = np.zeros((240, 320, 3), dtype=np.uint8)
        global_ = np.ones((240, 320, 3), dtype=np.uint8)

        result, _ = client.infer(7, state, wrist, global_, 10.0, 10.01, 10.004)

        self.assertEqual(len(captured), 5)
        header = json.loads(captured[1])
        self.assertEqual(header["wrist_timestamp"], 10.0)
        self.assertEqual(header["global_timestamp"], 10.01)
        self.assertAlmostEqual(header["observation_timestamp"], 10.005)
        self.assertEqual(header["state_timestamp"], 10.004)
        np.testing.assert_array_equal(
            np.frombuffer(captured[4], dtype=np.uint8).reshape(240, 320, 3),
            global_,
        )
        np.testing.assert_array_equal(result, action)

    def test_quest_tac_contract_is_announced(self):
        client = PolicyClient(
            "ipc:///unused", state_dim=7, image_shape=(480, 640, 3)
        )
        captured = []

        def exchange(parts):
            captured.extend(parts)
            return [
                MAGIC,
                json.dumps(
                    {
                        "request_id": 3,
                        "shape": [1, 7],
                        "observation_timestamp": 1.005,
                        "policy_type": "act",
                        "execution_steps": 1,
                        "temporal_ensemble_coeff": 0.01,
                        "period_s": 0.1,
                    }
                ).encode(),
                np.zeros((1, 7), dtype=np.float32).tobytes(),
            ]

        client._exchange = exchange
        state = np.zeros(7, dtype=np.float32)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        client.infer(3, state, rgb, rgb, 1.0, 1.01, 1.004)
        header = json.loads(captured[1])
        self.assertEqual(header["state_dim"], 7)
        self.assertEqual(header["image_shape"], [480, 640, 3])


if __name__ == "__main__":
    unittest.main()

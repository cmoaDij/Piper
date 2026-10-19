import unittest

from piper_policy_bridge.camera_sync import (
    NearestCameraSynchronizer,
    TimestampedImage,
    select_pair_state,
)


class CameraSyncTest(unittest.TestCase):
    @staticmethod
    def frame(role, timestamp):
        return TimestampedImage(role, role, timestamp, timestamp + 0.01)

    def test_bridge_uses_mutually_nearest_buffered_pair(self):
        synchronizer = NearestCameraSynchronizer(0.025)
        synchronizer.push(self.frame("wrist", 1.0))
        synchronizer.push(self.frame("global", 1.020))
        synchronizer.push(self.frame("wrist", 1.022))
        pairs = synchronizer.push(self.frame("global", 1.055))

        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0][0].timestamp, 1.022)
        self.assertAlmostEqual(pairs[0][1].timestamp, 1.020)
        self.assertEqual(synchronizer.unmatched["wrist"], 1)

    def test_bridge_rejects_out_of_order_camera_timestamp(self):
        synchronizer = NearestCameraSynchronizer(0.025)
        synchronizer.push(self.frame("wrist", 2.0))
        with self.assertRaises(ValueError):
            synchronizer.push(self.frame("wrist", 1.9))

    def test_pair_uses_same_nearest_state_snapshot_rule_as_collection(self):
        wrist = TimestampedImage("wrist", "w", 1.000, 1.01, "wrist-state", 0.980)
        global_ = TimestampedImage("global", "g", 1.020, 1.03, "global-state", 1.005)

        selected = select_pair_state(wrist, global_, 0.05)

        self.assertIsNotNone(selected)
        self.assertEqual(selected.state, "global-state")
        self.assertAlmostEqual(selected.timestamp, 1.005)
        self.assertAlmostEqual(selected.image_skew, -0.005)

    def test_pair_rejects_state_outside_collection_skew_limit(self):
        wrist = TimestampedImage("wrist", "w", 2.000, 2.01, "state", 1.940)
        global_ = TimestampedImage("global", "g", 2.020, 2.03, "state", 1.940)

        self.assertIsNone(select_pair_state(wrist, global_, 0.05))

    def test_pair_rejects_missing_state_snapshot(self):
        wrist = self.frame("wrist", 3.000)
        global_ = self.frame("global", 3.010)

        self.assertIsNone(select_pair_state(wrist, global_, 0.05))


if __name__ == "__main__":
    unittest.main()

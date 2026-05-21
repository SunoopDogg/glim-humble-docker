"""Trim a recorded bag so every kept RGB frame has a pose near its timestamp.

Core rule (pure, host-unit-tested): keep an RGB frame iff a pose exists within
+/- tolerance of its stamp. This drops the three pose-less segments in one pass --
the front (before icp converged), mid-run gaps (icp tracking loss), and the tail
(after the last pose). Pose messages are kept verbatim.

`keep_rgb` is import-safe without ROS. The `main` CLI lazily imports rosbag2_py so
the pure matcher can be unit-tested under a bare `uv` venv.
"""
import argparse
import bisect


def keep_rgb(rgb_t, pose_times_sorted, tol_ns):
    """True iff some pose timestamp is within +/- tol_ns of rgb_t.

    rgb_t, tol_ns: int nanoseconds. pose_times_sorted: ascending list of int ns.
    """
    if not pose_times_sorted:
        return False
    i = bisect.bisect_left(pose_times_sorted, rgb_t)
    # Nearest pose is at index i (>= rgb_t) or i-1 (< rgb_t); check both.
    for j in (i, i - 1):
        if 0 <= j < len(pose_times_sorted):
            if abs(pose_times_sorted[j] - rgb_t) <= tol_ns:
                return True
    return False


def _collect_pose_times(uri, storage_id, pose_topic):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id=storage_id),
                ConverterOptions('', ''))
    times = []
    while reader.has_next():
        topic, _data, t = reader.read_next()
        if topic == pose_topic:
            times.append(t)
    times.sort()
    return times


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--in', dest='in_uri', required=True, help='input bag dir')
    parser.add_argument('--out', dest='out_uri', required=True,
                        help='output bag dir (must NOT exist)')
    parser.add_argument('--pose-topic', default='/go2/map_pose')
    parser.add_argument('--rgb-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--tolerance', type=float, default=0.1,
                        help='max |rgb_stamp - pose_stamp| to keep an RGB frame (s)')
    parser.add_argument('--storage-id', default='sqlite3')
    ns = parser.parse_args(args)

    from rosbag2_py import (
        SequentialReader, SequentialWriter, StorageOptions, ConverterOptions,
    )

    tol_ns = int(ns.tolerance * 1e9)
    pose_times = _collect_pose_times(ns.in_uri, ns.storage_id, ns.pose_topic)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=ns.in_uri, storage_id=ns.storage_id),
                ConverterOptions('', ''))
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=ns.out_uri, storage_id=ns.storage_id),
                ConverterOptions('', ''))
    for topic_meta in reader.get_all_topics_and_types():
        writer.create_topic(topic_meta)

    kept = dropped = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == ns.rgb_topic and not keep_rgb(t, pose_times, tol_ns):
            dropped += 1
            continue
        if topic == ns.rgb_topic:
            kept += 1
        writer.write(topic, data, t)

    print(f'trim: kept {kept} RGB, dropped {dropped} pose-less RGB '
          f'(tol={ns.tolerance}s, {len(pose_times)} poses) -> {ns.out_uri}')


if __name__ == '__main__':
    main()

"""MCAP round trip and PointCloud2 <-> numpy helpers."""

from pathlib import Path

import numpy as np
import pytest

from mss_sim.bag_io import (
    TYPESTORE,
    BagReader,
    BagWriter,
    CloudArrays,
    CloudFormatError,
    arrays_to_cloud,
    cloud_to_arrays,
    make_header,
    stamp_ns,
)

_IMU = "sensor_msgs/msg/Imu"
_CLOUD = "sensor_msgs/msg/PointCloud2"


def _imu(stamp: int) -> object:
    v3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    quat = TYPESTORE.types["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0)
    cov = np.zeros(9)
    return TYPESTORE.types[_IMU](
        header=make_header(stamp, "imu"),
        orientation=quat,
        orientation_covariance=cov,
        angular_velocity=v3(x=0.1, y=0.2, z=0.3),
        angular_velocity_covariance=cov,
        linear_acceleration=v3(x=0.0, y=0.0, z=9.8),
        linear_acceleration_covariance=cov,
    )


def _arrays(h: int = 16, w: int = 720) -> CloudArrays:
    rng = np.random.default_rng(7)
    n = h * w
    return CloudArrays(
        xyz=rng.uniform(-50, 50, (n, 3)).astype(np.float32),
        intensity=rng.uniform(0, 255, n).astype(np.float32),
        t_ns=rng.integers(0, 100_000_000, n).astype(np.uint32),
        ring=(np.arange(n) % h).astype(np.uint16),
        height=h,
        width=w,
    )


def test_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "bag"
    arr = _arrays()
    with BagWriter(path) as w:
        w.add_topic("/imu", _IMU)
        w.add_topic("/lidar", _CLOUD)
        for t in (3_000, 1_000, 2_000):
            w.write("/imu", t, _imu(t))
        w.write("/lidar", 1_500, arrays_to_cloud(arr, 1_500, "lidar"))
    with BagReader(path) as r:
        assert r.topics() == {"/imu": _IMU, "/lidar": _CLOUD}
        msgs = list(r.messages(["/imu", "/lidar"]))
    assert len(msgs) == 4
    times = [m[1] for m in msgs]
    assert times == sorted(times)
    assert [m[0] for m in msgs] == ["/imu", "/lidar", "/imu", "/imu"]
    assert stamp_ns(msgs[0][2].header) == 1_000
    cloud = msgs[1][2]
    assert cloud.point_step == 22 and not cloud.is_dense
    out = cloud_to_arrays(cloud)
    assert out.height == 16 and out.width == 720
    assert np.allclose(out.xyz, arr.xyz)
    assert np.array_equal(out.t_ns, arr.t_ns)
    assert np.array_equal(out.ring, arr.ring)
    assert np.allclose(out.intensity, arr.intensity)
    assert out.xyz.dtype == np.float32


def test_topic_filter(tmp_path: Path) -> None:
    path = tmp_path / "bag"
    with BagWriter(path) as w:
        w.add_topic("/imu", _IMU)
        w.add_topic("/imu2", _IMU)
        w.write("/imu", 1, _imu(1))
        w.write("/imu2", 2, _imu(2))
    with BagReader(path) as r:
        assert [m[0] for m in r.messages(["/imu2"])] == ["/imu2"]


def test_xyz_only_and_field_order() -> None:
    pf = TYPESTORE.types["sensor_msgs/msg/PointField"]
    pts = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
    # Fields deliberately out of order with padding: z, x, y at offsets 0, 8, 4 of a 16-byte step.
    data = np.zeros(2, dtype=[("z", "<f4"), ("y", "<f4"), ("x", "<f4"), ("pad", "<f4")])
    data["x"], data["y"], data["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    msg = TYPESTORE.types["sensor_msgs/msg/PointCloud2"](
        header=make_header(5, "f"),
        height=1,
        width=2,
        fields=[
            pf(name=n, offset=o, datatype=7, count=1) for n, o in (("z", 0), ("y", 4), ("x", 8))
        ],
        is_bigendian=False,
        point_step=16,
        row_step=32,
        data=np.frombuffer(data.tobytes(), dtype=np.uint8),
        is_dense=True,
    )
    out = cloud_to_arrays(msg)
    assert np.allclose(out.xyz, pts)
    assert out.t_ns is None and out.ring is None and out.intensity is not None
    assert (out.height, out.width) == (1, 2)


def test_missing_t_ring_written_as_zero() -> None:
    a = _arrays(2, 5)
    a.t_ns = None
    a.ring = None
    out = cloud_to_arrays(arrays_to_cloud(a, 9, "x"))
    assert np.all(out.t_ns == 0) and np.all(out.ring == 0)


def test_stamp() -> None:
    h = make_header(1_234_567_890_123, "f")
    assert (h.stamp.sec, h.stamp.nanosec) == (1234, 567_890_123)
    assert stamp_ns(h) == 1_234_567_890_123 and h.frame_id == "f"


def test_unknown_and_empty_topics(tmp_path: Path) -> None:
    path = tmp_path / "bag"
    with BagWriter(path) as w:
        w.add_topic("/imu", _IMU)
        w.write("/imu", 1, _imu(1))
    with BagReader(path) as r:
        with pytest.raises(KeyError, match="/typo"):
            r.messages(["/imu", "/typo"])
        assert list(r.messages([])) == []


def test_malformed_clouds_raise_cloud_format_error() -> None:
    short = arrays_to_cloud(_arrays(2, 5), 1, "f")
    short.data = short.data[:-1]
    with pytest.raises(CloudFormatError, match="too short"):
        cloud_to_arrays(short)
    no_z = arrays_to_cloud(_arrays(2, 5), 1, "f")
    no_z.fields = [f for f in no_z.fields if f.name != "z"]
    with pytest.raises(CloudFormatError, match="z"):
        cloud_to_arrays(no_z)

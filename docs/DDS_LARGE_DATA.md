# Large point clouds over local UDP

The UDP-only profile requests 4 MiB send and receive socket buffers. Before
starting the ROS services, check the actual Linux socket capacity:

```sh
python3 scripts/dds_buffer_limits.py
```

If the check reports `ready: false`, an administrator can explicitly raise the
maximum buffer limits, with a rollback record:

```sh
sudo python3 scripts/dds_buffer_limits.py --apply
```

This changes `net.core.rmem_max` and `net.core.wmem_max` to at least 4 MiB. It
preserves larger existing maximums, both socket defaults, interface settings,
routes and clocks. Existing sockets keep their old allocations. Restart the
relevant sensor/navigation ROS services, then check actual DDS socket sizes and
source-header ages. The script never starts or stops robot-control processes.

The profile retains UDP-only transport and the current publication mode. It
does not disable reliable delivery, alter source stamps, widen freshness gates,
or configure PTP. A buffer change cannot prove end-to-end latency or remove the
need to inspect actual sensor acquisition timestamps.

The supported XML fields are documented in the
[Fast DDS 2.6 UDP transport reference](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/udp/udp.html).
Linux reports double the requested socket capacity to include kernel overhead;
the check validates this returned value, rather than just reading a config file.

## Recorded observation, 2026-10-31

A stationary deployment used approximately 400 kB serialized LiDAR messages,
but the host's maximum/default receive buffer was 212,992 bytes. In a bounded
20 s probe, 1,237 global UDP receive-buffer errors accumulated, and odometry
ROS-header-to-receipt age had median 1.197 s and p95 1.919 s.

After raising only the maximum caps and explicit DDS buffers, a new 20 s probe
under navigation-preview load measured odometry median 0.040 s, p95 0.059 s,
maximum 0.063 s and zero odometry samples over 0.5 s. The services were restarted
and the second probe overlapped navigation startup; this is operational
before/after evidence, not a fully controlled attribution experiment. Global
UDP receive-buffer errors still increased by 129 and are not claimed eliminated
or all attributed to LiDAR. Startup discovery also affects received sample
counts. Keep the original failures and perform sustained validation.

The sensor was using host-receive-derived ROS headers without synchronized
device time. These numbers are ROS message header ages, not device sampling
latency, mapping accuracy, obstacle stopping or physical navigation acceptance.

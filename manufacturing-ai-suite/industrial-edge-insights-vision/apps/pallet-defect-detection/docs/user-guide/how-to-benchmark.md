# How to Benchmark Pallet Defect Detection
This guide demonstrates how to benchmark the pallet defect detection pipeline to determine optimal stream density and performance characteristics.

## Contents

### Prerequisites
> Ensure the application is set up and running. Refer to the [Setup Guide](../setup-guide.md) for complete installation and configuration steps.

- DL Streamer Pipeline Server (DLSPS) running and accessible
- `curl`, `jq`, and `bc` utilities installed

### Benchmark Script Usage

Navigate to the `[WORKDIR]/edge-ai-suites/manufacturing-ai-suite/industrial-edge-insights-vision` directory and use the benchmark script:

```bash
./benchmark_start.sh -p <payload_file> -l <lower_bound> -u <upper_bound> [-t <target_fps>] [-i <interval>]
```

**Arguments:**
- `-p <payload_file>` : **(Required)** Path to the benchmark payload JSON file
- `-l <lower_bound>` : **(Required)** Starting lower bound for number of streams
- `-u <upper_bound>` : **(Required)** Starting upper bound for number of streams  
- `-t <target_fps>` : Target FPS threshold (default: 28.5)
- `-i <interval>` : Monitoring duration in seconds per test run (default: 60)

### Payload Files

Choose the appropriate payload file based on your benchmarking objective:

1. **App's Best Performance (`gpu_payload.json`)**
   - Uses GPU device with optimized settings and WebRTC output
   - Tests realistic application-level performance with GPU acceleration
   - Includes end-to-end pipeline with output destination

2. **GPU's Best Performance (`benchmark_gpu_payload.json`)**  
   - Uses GPU device with optimized batch processing
   - Tests maximum GPU throughput capabilities
   - Hardware-accelerated preprocessing without output overhead for pure performance testing

### Steps to run benchmarks

1. Test the app's best performance with GPU acceleration:
   ```bash
   ./benchmark_start.sh -p apps/pallet-defect-detection/gpu_payload.json -l 1 -u 10 -t 25.0 -i 30
   ```

2. Test maximum GPU performance for pure throughput:
   ```bash
   ./benchmark_start.sh -p apps/pallet-defect-detection/benchmark_gpu_payload.json -l 1 -u 20 -t 28.5 -i 60
   ```

   > NOTE: Replace the payload file path with the actual path to your benchmark payload files.

### Understanding Results

The benchmark uses binary search to find optimal stream density. Key metrics include:

- **Stream Density**: Maximum concurrent streams achieving target FPS
- **Throughput Median**: Median FPS across all streams  
- **Throughput Cumulative**: Total FPS sum of all streams

**Sample Output:**
```
streams: 6 throughput: 28.8 range: [5,7]
======================================================
✅ FINAL RESULT: Stream-Density Benchmark Completed!
stream density: 6
======================================================
throughput median: 29.0
throughput cumulative: 173.8
```

### Troubleshooting

**Common Issues:**

1. **DLSPS Not Accessible**
   ```
   Error: DL Streamer Pipeline Server is not running or not reachable
   ```
   - Verify DLSPS is running: `docker ps | grep dlstreamer`
   - Check network connectivity to localhost

2. **Pipeline Startup Failures**
   - Check model file paths in payload
   - Verify video file accessibility  
   - Monitor system resources (CPU, memory, GPU)

3. **Debug Mode**
   Add `--trace` to see detailed execution steps:
   ```bash
   ./benchmark_start.sh -p gpu_payload.json -l 1 -u 10 --trace
   ```

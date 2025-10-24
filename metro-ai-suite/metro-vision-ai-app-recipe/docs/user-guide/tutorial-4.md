# AI Crowd Analytics Tutorial

<!--
**Sample Description**: This tutorial demonstrates how to build an intelligent crowd analytics system using edge AI technologies for real-time vehicle detection and crowd identification in parking lots.
-->

This tutorial walks you through creating an AI-powered crowd analytics system that automatically detects vehicles and identifies whether they form a "crowd" (closely grouped vehicles) or are scattered individually. The system leverages Intel's DLStreamer framework with pre-trained AI models to process video streams and analyze vehicle clustering patterns in real-time.

<!--
**What You Can Do**: This guide covers the complete development workflow for building an AI crowd analytics application.
-->

By following this guide, you will learn how to:
- **Set up the Crowd Analytics Application**: Create a new application based on the Smart Parking template and configure it for crowd detection use cases
- **Download and Configure AI Models**: Install YOLO object detection models and custom vehicle classification models
- **Configure Video Processing Pipeline**: Set up the DLStreamer pipeline for real-time vehicle detection and crowd analysis
- **Deploy and Run the System**: Launch the containerized application and monitor crowd detection performance

## Prerequisites

- Verify that your system meets the [minimum system requirements](./system-requirements.md) for running edge AI applications
- Install Docker: [Docker Installation Guide](https://docs.docker.com/get-docker/)
- Enable running Docker without "sudo": [Post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/)
- Ensure you have at least 8GB of available disk space for AI models and video files
- Basic understanding of containerized applications and video processing concepts

## Application Architecture Overview

<!--
**Architecture Image Placeholder**: Add architecture diagram showing the flow from video input through AI models to toll processing output
-->
![Crowd Analytics System Diagram](_images/ai-tolling-system.svg)


The AI Crowd Analytics system consists of several key components:
- **Video Input**: Processes live camera feeds or video files from parking lot cameras
- **Vehicle Detection**: Uses YOLOv10s model to detect and track vehicles in the parking lot
- **Vehicle Tracking**: Maintains consistent vehicle identification across video frames
- **Crowd Detection Algorithm**: Analyzes vehicle positions using Euclidean distance calculations in Node-RED to identify clusters
- **Data Processing**: Determines whether detected vehicles form a "crowd" (closely grouped) or are scattered individually
- **Real-time Analytics**: Provides live updates on crowd formation and dispersal patterns

## Set up and First Use

### 1. **Create the Crowd Analytics Application Directory**

Navigate to the metro vision AI recipe directory and create the crowd-analytics application by copying the Smart Parking template:

```bash
cd ./edge-ai-suites/metro-ai-suite/metro-vision-ai-app-recipe
cp -r smart-parking/ crowd-analytics/
```

This creates a new `crowd-analytics` directory with all the necessary application structure and configuration files.

### 2. **Download Sample Video File**

Download a sample video file containing vehicle traffic for testing the AI tolling system:

```bash
mkdir -p ./crowd-analytics/src/dlstreamer-pipeline-server/videos/
wget -O ./crowd-analytics/src/dlstreamer-pipeline-server/videos/easy1.mp4 \
  https://github.com/freedomwebtech/yolov8-advance-parkingspace-detection/raw/main/easy1.mp4

```

<details>
<summary>
Video File Details
</summary>

The sample video contains:
- Multiple vehicles in various parking scenarios
- Examples of both clustered (crowded) and scattered vehicle arrangements
- Duration: Approximately 21 seconds of footage

</details>

### 3. **Download and Setup AI Models**

Create and run the model download script to install all required AI models:

```bash
docker run --rm --user=root \
  -e http_proxy -e https_proxy -e no_proxy \
  -v "$PWD:/home/dlstreamer/metro-suite" \
  intel/dlstreamer:2025.1.2-ubuntu24 bash -c "$(cat <<EOF

cd /home/dlstreamer/metro-suite/

mkdir -p crowd-analytics/src/dlstreamer-pipeline-server/models/public
export MODELS_PATH=/home/dlstreamer/metro-suite/crowd-analytics/src/dlstreamer-pipeline-server/models
/home/dlstreamer/dlstreamer/samples/download_public_models.sh yolo11s coco128

# mkdir -p crowd-analytics/src/dlstreamer-pipeline-server/models/intel

echo "Fix ownership..."
chown -R "$(id -u):$(id -g)" crowd-analytics/src/dlstreamer-pipeline-server/models crowd-analytics/src/dlstreamer-pipeline-server/videos 2>/dev/null || true
EOF
)"
```

The installation script downloads and sets up essential AI models:

| **Model Name** | **Purpose** | **Framework** | **Size** |
|----------------|-------------|---------------|----------|
| YOLOv10s | Vehicle detection and localization | PyTorch/OpenVINO | ~20MB |

<details>
<summary>
Model Download Process Details
</summary>

The installation script performs the following operations:
1. Creates the required directory structure under `src/dlstreamer-pipeline-server/models/`
2. Runs a DLStreamer container to access model download tools
3. Downloads public YOLO models using the built-in download scripts
4. Sets up proper file permissions for container access

Expected download time: 5-10 minutes depending on internet connection.

</details>

### 4. **Configure the AI Processing Pipeline**

Update the pipeline configuration to use the crowd analytics AI models. Create or update the configuration file:

```bash
cat > ./crowd-analytics/src/dlstreamer-pipeline-server/config.json << 'EOF'
{
    "config": {
        "pipelines": [
            {
                "name": "yolov11s_crowd_analytics",
                "source": "gstreamer",
                "queue_maxsize": 50,
                "pipeline": "{auto_source} name=source ! decodebin ! gvadetect model=/home/pipeline-server/models/public/yolo11s/INT8/yolo11s.xml device=CPU pre-process-backend=opencv name=detection ! queue ! gvatrack tracking-type=short-term-imageless ! queue ! gvametaconvert add-empty-results=true name=metaconvert ! queue ! gvafpscounter ! appsink name=destination",
                "description": "Vehicle detection and tracking for crowd analytics",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "detection-properties": {
                            "element": {
                                "name": "detection",
                                "format": "element-properties"
                            }
                        }
                    }
                },
                "auto_start": false
            }
        ]
    }
}
EOF
```
<details>
<summary>
Pipeline Configuration Explanation
</summary>

The GStreamer pipeline configuration defines the crowd analytics AI processing workflow:

- **Source**: Accepts video input from parking lot camera feeds or video files
- **Decode**: Converts video format to raw frames for processing  
- **gvadetect**: Runs YOLOv10s object detection to identify vehicles in the parking lot
- **gvatrack**: Tracks detected vehicles across frames using zero-term tracking for stability
- **gvawatermark**: Adds visual annotations showing detected vehicles and bounding boxes
- **gvametaconvert**: Converts inference results to structured metadata with vehicle positions
- **gvametapublish**: Publishes vehicle detection data to MQTT for crowd analysis in Node-RED
- **gvafpscounter**: Monitors processing performance

The crowd detection logic (analyzing vehicle clustering and proximity) is handled downstream in Node-RED using the published vehicle position data.

</details>
### 5. **Configure Application Environment**

Update the environment configuration to use the Crowd Analytics application:

```bash
# Update the .env file to specify the crowd-analytics application and HOST IP Address
sed -i 's/^SAMPLE_APP=.*/SAMPLE_APP=crowd-analytics/' .env
sed -i "s/^HOST_IP=.*/HOST_IP=$(hostname -I | cut -f1 -d' ')/" .env


# Create self signed certificate for nginx
mkdir -p src/nginx/ssl
cd crowd-analytics/src/nginx/ssl
if [ ! -f server.key ] || [ ! -f server.crt ]; then
    echo "Generate self-signed certificate..."
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout server.key -out server.crt -subj "/C=US/ST=CA/L=San Francisco/O=Intel/OU=Edge AI/CN=localhost"
    chown -R "$(id -u):$(id -g)" server.key server.crt 2>/dev/null || true

fi

# Verify the configuration
grep SAMPLE_APP= .env
grep HOST_IP= .env
```

Expected output: `SAMPLE_APP=crowd-analytics`

### 6. **Deploy the Application**

Set up the Docker Compose configuration and start the application:

```bash
# Copy the compose file for deployment
cp compose-without-scenescape.yml docker-compose.yml

# Start all services in detached mode
docker compose up -d
```

The deployment process will:
- Pull required container images
- Start the DLStreamer pipeline server
- Initialize the Node-RED flow management
- Launch the Grafana dashboard
- Set up the MQTT message broker

## Validation and Expected Results

### 1. **Verify Service Status**

Check that all services are running correctly:

```bash
docker ps
```

Expected output should show containers for:
- `dlstreamer-pipeline-server`
- `node-red`
- `grafana`
- `mosquitto` (MQTT broker)

### 2. **Access the Application Interface**

Open your web browser and navigate to:
- **Main Dashboard**: `https://localhost/grafana` (Grafana)
    - Username: admin
    - Password: admin
- **Node-RED Flow Editor**: `https://localhost/nodered/`

### 3. **Test Video Processing**

Start the AI pipeline and process the sample video:

```bash
# Start the crowd analytics pipeline with the easy1.mp4 video (basic mode)
curl -k -s https://localhost/api/pipelines/user_defined_pipelines/yolov11s_crowd_analytics -X POST -H 'Content-Type: application/json' -d '
{
    "source": {
        "uri": "file:///home/pipeline-server/videos/easy1.mp4",
        "type": "uri"
    },
    "destination": {
        "metadata": {
            "type": "mqtt",
            "topic": "object_detection_1",
            "timeout": 1000
        },
        "frame": {
            "type": "webrtc",
            "peer-id": "object_detection_1"
        }
    },
    "parameters": {
        "detection-device": "CPU"
    }
}'
```

### 4. **View Live Video Stream**

Access the processed video stream with AI annotations through WebRTC:

```bash
# Open in your web browser (replace <HOST_IP> with your actual IP address)
# For local testing, typically use localhost or 127.0.0.1
http://<HOST_IP>:8889/object_detection_1
```

For local testing, you can use: `http://localhost:8889/object_detection_1`

![Crowd Analytics Live Detection](_images/crowd_analytics_detection.jpg)

Expected results:
- Vehicle detection accuracy > 90% in parking lot scenarios
- Real-time vehicle tracking with consistent IDs across frames
- Crowd detection analysis showing grouped vs scattered vehicles
- Live video stream with bounding boxes and crowd indicators
- Processing at 15-30 FPS depending on hardware capabilities

### 5. **Understanding Crowd Detection Logic**

The crowd detection algorithm works through the following process:

1. **Vehicle Detection**: YOLOv10s identifies all vehicles in each video frame with bounding boxes
2. **Vehicle Tracking**: Maintains consistent vehicle IDs across frames using zero-term tracking
3. **Position Analysis**: Calculates centroid coordinates for each detected vehicle
4. **Distance Calculation**: Node-RED computes Euclidean distances between all vehicle pairs
5. **Clustering Decision**: Applies configurable distance threshold to determine crowd formation:
   - Vehicles within distance threshold = "crowd" (closely grouped)
   - Vehicles beyond distance threshold = "scattered" (individually parked)
6. **Real-time Updates**: Continuously analyzes vehicle arrangements frame by frame

<details>
<summary>
Crowd Detection Parameters
</summary>

Key configurable parameters in Node-RED for crowd analysis:
- **distance_threshold**: 400 pixels - minimum distance to consider vehicles as separate
- **intersection_threshold**: 0.85 - overlap threshold for bounding box analysis  
- **object_confidence**: 0.5 - minimum confidence for vehicle detection
- **target_object**: ["car", "truck", "bus"] - vehicle types to analyze for crowds

These parameters can be fine-tuned in the Node-RED interface based on:
- Camera height and angle
- Parking lot layout and size
- Desired crowd sensitivity
- Vehicle density thresholds

</details>
## Troubleshooting

### 1. **Container Startup Issues**

If containers fail to start:
```bash
# Check container logs for specific errors
docker logs <container_name>

# Common issues:
# - Port conflicts: Ensure ports 3000, 1880, 8080 are available
# - Permission issues: Verify Docker permissions
# - Resource constraints: Check available memory and disk space
```

### 2. **Model Download Failures**

If model download fails during installation:
```bash
# Retry the installation with verbose output
./install.sh 2>&1 | tee install.log

# Check for network connectivity issues
curl -I https://github.com/openvinotoolkit/open_model_zoo

# Verify disk space
df -h
```

### 3. **Pipeline Processing Errors**

If video processing fails or shows poor accuracy:
```bash
# Check pipeline server logs
docker logs dlstreamer-pipeline-server

# Verify model files are properly installed
ls -la ./crowd-analytics/src/dlstreamer-pipeline-server/models/

# Test with different video source
# Replace the video file with a different sample
```

### 4. **Performance Issues**

For slow processing or high CPU usage:
- **Reduce video resolution**: Use lower resolution input videos
- **Adjust inference device**: Change from CPU to GPU if available
- **Optimize pipeline**: Reduce queue sizes or disable unnecessary features

## Next Steps

After successfully setting up the AI Tolling system, consider these enhancements:

[**Integration with Node Red for enhancing business logic**](./tutorial-2.md)


## Supporting Resources

- [DLStreamer Documentation](https://dlstreamer.github.io/)
- [Metro AI Solutions](https://github.com/open-edge-platform/edge-ai-suites/tree/main/metro-ai-suite)